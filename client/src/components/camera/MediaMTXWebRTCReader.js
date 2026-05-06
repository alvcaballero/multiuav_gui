/* eslint-disable */
/** WebRTC/WHEP reader for MediaMTX. */
export class MediaMTXWebRTCReader {
  constructor(conf) {
    this.retryPause = 2000;
    this.conf = conf;
    this.state = 'getting_codecs';
    this.restartTimeout = null;
    this.pc = null;
    this.offerData = null;
    this.sessionUrl = null;
    this.queuedCandidates = [];
    this.#getNonAdvertisedCodecs();
  }

  close() {
    this.state = 'closed';
    if (this.pc !== null) {
      this.pc.close();
    }
    if (this.restartTimeout !== null) {
      clearTimeout(this.restartTimeout);
    }
  }

  static #supportsNonAdvertisedCodec(codec, fmtp) {
    return new Promise((resolve) => {
      const pc = new RTCPeerConnection({ iceServers: [] });
      const mediaType = 'audio';
      let payloadType = '';

      pc.addTransceiver(mediaType, { direction: 'recvonly' });
      pc.createOffer()
        .then((offer) => {
          if (offer.sdp === undefined) throw new Error('SDP not present');
          if (offer.sdp.includes(` ${codec}`)) throw new Error('already present');

          const sections = offer.sdp.split(`m=${mediaType}`);
          const payloadTypes = sections.slice(1)
            .map((s) => s.split('\r\n')[0].split(' ').slice(3))
            .reduce((prev, cur) => [...prev, ...cur], []);
          payloadType = this.#reservePayloadType(payloadTypes);

          const lines = sections[1].split('\r\n');
          lines[0] += ` ${payloadType}`;
          lines.splice(lines.length - 1, 0, `a=rtpmap:${payloadType} ${codec}`);
          if (fmtp !== undefined) {
            lines.splice(lines.length - 1, 0, `a=fmtp:${payloadType} ${fmtp}`);
          }
          sections[1] = lines.join('\r\n');
          offer.sdp = sections.join(`m=${mediaType}`);
          return pc.setLocalDescription(offer);
        })
        .then(() => (
          pc.setRemoteDescription(new RTCSessionDescription({
            type: 'answer',
            sdp: 'v=0\r\n'
            + 'o=- 6539324223450680508 0 IN IP4 0.0.0.0\r\n'
            + 's=-\r\n'
            + 't=0 0\r\n'
            + 'a=fingerprint:sha-256 0D:9F:78:15:42:B5:4B:E6:E2:94:3E:5B:37:78:E1:4B:54:59:A3:36:3A:E5:05:EB:27:EE:8F:D2:2D:41:29:25\r\n'
            + `m=${mediaType} 9 UDP/TLS/RTP/SAVPF ${payloadType}\r\n`
            + 'c=IN IP4 0.0.0.0\r\n'
            + 'a=ice-pwd:7c3bf4770007e7432ee4ea4d697db675\r\n'
            + 'a=ice-ufrag:29e036dc\r\n'
            + 'a=sendonly\r\n'
            + 'a=rtcp-mux\r\n'
            + `a=rtpmap:${payloadType} ${codec}\r\n`
            + ((fmtp !== undefined) ? `a=fmtp:${payloadType} ${fmtp}\r\n` : ''),
          }))
        ))
        .then(() => resolve(true))
        .catch(() => resolve(false))
        .finally(() => pc.close());
    });
  }

  static #unquoteCredential(v) {
    return JSON.parse(`"${v}"`);
  }

  static #linkToIceServers(links) {
    return (links !== null) ? links.split(', ').map((link) => {
      const m = link.match(/^<(.+?)>; rel="ice-server"(; username="(.*?)"; credential="(.*?)"; credential-type="password")?/i);
      const ret = { urls: [m[1]] };
      if (m[3] !== undefined) {
        ret.username = this.#unquoteCredential(m[3]);
        ret.credential = this.#unquoteCredential(m[4]);
        ret.credentialType = 'password';
      }
      return ret;
    }) : [];
  }

  static #parseOffer(sdp) {
    const ret = { iceUfrag: '', icePwd: '', medias: [] };
    for (const line of sdp.split('\r\n')) {
      if (line.startsWith('m=')) {
        ret.medias.push(line.slice('m='.length));
      } else if (ret.iceUfrag === '' && line.startsWith('a=ice-ufrag:')) {
        ret.iceUfrag = line.slice('a=ice-ufrag:'.length);
      } else if (ret.icePwd === '' && line.startsWith('a=ice-pwd:')) {
        ret.icePwd = line.slice('a=ice-pwd:'.length);
      }
    }
    return ret;
  }

  static #reservePayloadType(payloadTypes) {
    for (let i = 30; i <= 127; i++) {
      if ((i <= 63 || i >= 96) && !payloadTypes.includes(i.toString())) {
        const pl = i.toString();
        payloadTypes.push(pl);
        return pl;
      }
    }
    throw Error('unable to find a free payload type');
  }

  static #enableStereoPcmau(payloadTypes, section) {
    const lines = section.split('\r\n');
    let payloadType = this.#reservePayloadType(payloadTypes);
    lines[0] += ` ${payloadType}`;
    lines.splice(lines.length - 1, 0, `a=rtpmap:${payloadType} PCMU/8000/2`);
    lines.splice(lines.length - 1, 0, `a=rtcp-fb:${payloadType} transport-cc`);
    payloadType = this.#reservePayloadType(payloadTypes);
    lines[0] += ` ${payloadType}`;
    lines.splice(lines.length - 1, 0, `a=rtpmap:${payloadType} PCMA/8000/2`);
    lines.splice(lines.length - 1, 0, `a=rtcp-fb:${payloadType} transport-cc`);
    return lines.join('\r\n');
  }

  static #enableMultichannelOpus(payloadTypes, section) {
    const lines = section.split('\r\n');
    const configs = [
      { codec: 'multiopus/48000/3', fmtp: 'channel_mapping=0,2,1;num_streams=2;coupled_streams=1' },
      { codec: 'multiopus/48000/4', fmtp: 'channel_mapping=0,1,2,3;num_streams=2;coupled_streams=2' },
      { codec: 'multiopus/48000/5', fmtp: 'channel_mapping=0,4,1,2,3;num_streams=3;coupled_streams=2' },
      { codec: 'multiopus/48000/6', fmtp: 'channel_mapping=0,4,1,2,3,5;num_streams=4;coupled_streams=2' },
      { codec: 'multiopus/48000/7', fmtp: 'channel_mapping=0,4,1,2,3,5,6;num_streams=4;coupled_streams=4' },
      { codec: 'multiopus/48000/8', fmtp: 'channel_mapping=0,6,1,4,5,2,3,7;num_streams=5;coupled_streams=4' }
    ];
    configs.forEach(c => {
      let payloadType = this.#reservePayloadType(payloadTypes);
      lines[0] += ` ${payloadType}`;
      lines.splice(lines.length - 1, 0, `a=rtpmap:${payloadType} ${c.codec}`);
      lines.splice(lines.length - 1, 0, `a=fmtp:${payloadType} ${c.fmtp}`);
      lines.splice(lines.length - 1, 0, `a=rtcp-fb:${payloadType} transport-cc`);
    });
    return lines.join('\r\n');
  }

  static #enableL16(payloadTypes, section) {
    const lines = section.split('\r\n');
    [8000, 16000, 48000].forEach(rate => {
      let payloadType = this.#reservePayloadType(payloadTypes);
      lines[0] += ` ${payloadType}`;
      lines.splice(lines.length - 1, 0, `a=rtpmap:${payloadType} L16/${rate}/2`);
      lines.splice(lines.length - 1, 0, `a=rtcp-fb:${payloadType} transport-cc`);
    });
    return lines.join('\r\n');
  }

  static #enableStereoOpus(section) {
    let opusPayloadFormat = '';
    const lines = section.split('\r\n');
    for (let i = 0; i < lines.length; i++) {
      if (lines[i].startsWith('a=rtpmap:') && lines[i].toLowerCase().includes('opus/')) {
        opusPayloadFormat = lines[i].slice('a=rtpmap:'.length).split(' ')[0];
        break;
      }
    }
    if (opusPayloadFormat === '') return section;
    for (let i = 0; i < lines.length; i++) {
      if (lines[i].startsWith(`a=fmtp:${opusPayloadFormat} `)) {
        if (!lines[i].includes('stereo')) lines[i] += ';stereo=1';
        if (!lines[i].includes('sprop-stereo')) lines[i] += ';sprop-stereo=1';
      }
    }
    return lines.join('\r\n');
  }

  static #editOffer(sdp, nonAdvertisedCodecs) {
    const sections = sdp.split('m=');
    const payloadTypes = sections.slice(1)
      .map((s) => s.split('\r\n')[0].split(' ').slice(3))
      .reduce((prev, cur) => [...prev, ...cur], []);

    for (let i = 1; i < sections.length; i++) {
      if (sections[i].startsWith('audio')) {
        sections[i] = this.#enableStereoOpus(sections[i]);
        if (nonAdvertisedCodecs.includes('pcma/8000/2')) sections[i] = this.#enableStereoPcmau(payloadTypes, sections[i]);
        if (nonAdvertisedCodecs.includes('multiopus/48000/6')) sections[i] = this.#enableMultichannelOpus(payloadTypes, sections[i]);
        if (nonAdvertisedCodecs.includes('L16/48000/2')) sections[i] = this.#enableL16(payloadTypes, sections[i]);
        break;
      }
    }
    return sections.join('m=');
  }

  static #generateSdpFragment(od, candidates) {
    const candidatesByMedia = {};
    for (const candidate of candidates) {
      const mid = candidate.sdpMLineIndex;
      if (candidatesByMedia[mid] === undefined) candidatesByMedia[mid] = [];
      candidatesByMedia[mid].push(candidate);
    }
    let frag = `a=ice-ufrag:${od.iceUfrag}\r\n` + `a=ice-pwd:${od.icePwd}\r\n`;
    let mid = 0;
    for (const media of od.medias) {
      if (candidatesByMedia[mid] !== undefined) {
        frag += `m=${media}\r\n` + `a=mid:${mid}\r\n`;
        for (const candidate of candidatesByMedia[mid]) {
          frag += `a=${candidate.candidate}\r\n`;
        }
      }
      mid++;
    }
    return frag;
  }

  #handleError(err) {
    if (this.state === 'running') {
      if (this.pc !== null) { this.pc.close(); this.pc = null; }
      this.offerData = null;
      if (this.sessionUrl !== null) { fetch(this.sessionUrl, { method: 'DELETE' }); this.sessionUrl = null; }
      this.queuedCandidates = [];
      this.state = 'restarting';
      this.restartTimeout = window.setTimeout(() => {
        this.restartTimeout = null;
        this.state = 'running';
        this.#start();
      }, this.retryPause);
      if (this.conf.onError !== undefined) this.conf.onError(`${err}, retrying...`);
    } else if (this.state === 'getting_codecs') {
      this.state = 'failed';
      if (this.conf.onError !== undefined) this.conf.onError(err);
    }
  }

  #getNonAdvertisedCodecs() {
    Promise.all([
      ['pcma/8000/2'],
      ['multiopus/48000/6', 'channel_mapping=0,4,1,2,3,5;num_streams=4;coupled_streams=2'],
      ['L16/48000/2'],
    ].map((c) => MediaMTXWebRTCReader.#supportsNonAdvertisedCodec(c[0], c[1]).then((r) => ((r) ? c[0] : false))))
      .then((c) => c.filter((e) => e !== false))
      .then((codecs) => {
        if (this.state !== 'getting_codecs') throw new Error('closed');
        this.nonAdvertisedCodecs = codecs;
        this.state = 'running';
        this.#start();
      })
      .catch((err) => this.#handleError(err));
  }

  #start() {
    this.#requestICEServers()
      .then((iceServers) => this.#setupPeerConnection(iceServers))
      .then((offer) => this.#sendOffer(offer))
      .then((answer) => this.#setAnswer(answer))
      .catch((err) => this.#handleError(err.toString()));
  }

  #authHeader() {
    if (this.conf.user) return {'Authorization': `Basic ${btoa(`${this.conf.user}:${this.conf.pass}`)}`};
    if (this.conf.token) return {'Authorization': `Bearer ${this.conf.token}`};
    return {};
  }

  #requestICEServers() {
    return fetch(this.conf.url, { method: 'OPTIONS', headers: { ...this.#authHeader() } })
      .then((res) => MediaMTXWebRTCReader.#linkToIceServers(res.headers.get('Link')));
  }

  #setupPeerConnection(iceServers) {
    if (this.state !== 'running') throw new Error('closed');
    this.pc = new RTCPeerConnection({ iceServers, sdpSemantics: 'unified-plan' });
    const direction = 'recvonly';
    this.pc.addTransceiver('video', { direction });
    this.pc.addTransceiver('audio', { direction });
    this.pc.createDataChannel('');
    this.pc.onicecandidate = (evt) => this.#onLocalCandidate(evt);
    this.pc.onconnectionstatechange = () => this.#onConnectionState();
    this.pc.ontrack = (evt) => this.#onTrack(evt);
    this.pc.ondatachannel = (evt) => this.#onDataChannel(evt);

    return this.pc.createOffer()
      .then((offer) => {
        offer.sdp = MediaMTXWebRTCReader.#editOffer(offer.sdp, this.nonAdvertisedCodecs);
        this.offerData = MediaMTXWebRTCReader.#parseOffer(offer.sdp);
        return this.pc.setLocalDescription(offer).then(() => offer.sdp);
      });
  }

  #sendOffer(offer) {
    if (this.state !== 'running') throw new Error('closed');
    return fetch(this.conf.url, {
      method: 'POST',
      headers: { ...this.#authHeader(), 'Content-Type': 'application/sdp' },
      body: offer,
    })
      .then((res) => {
        if (res.status === 201) {
          this.sessionUrl = new URL(res.headers.get('location'), this.conf.url).toString();
          return res.text();
        }
        throw new Error(`status ${res.status}`);
      });
  }

  #setAnswer(answer) {
    if (this.state !== 'running') throw new Error('closed');
    return this.pc.setRemoteDescription(new RTCSessionDescription({ type: 'answer', sdp: answer }))
      .then(() => {
        if (this.state === 'running' && this.queuedCandidates.length !== 0) {
          this.#sendLocalCandidates(this.queuedCandidates);
          this.queuedCandidates = [];
        }
      });
  }

  #onLocalCandidate(evt) {
    if (this.state !== 'running' || !evt.candidate) return;
    if (this.sessionUrl === null) this.queuedCandidates.push(evt.candidate);
    else this.#sendLocalCandidates([evt.candidate]);
  }

  #sendLocalCandidates(candidates) {
    fetch(this.sessionUrl, {
      method: 'PATCH',
      headers: { 'Content-Type': 'application/trickle-ice-sdpfrag', 'If-Match': '*' },
      body: MediaMTXWebRTCReader.#generateSdpFragment(this.offerData, candidates),
    });
  }

  #onConnectionState() {
    if (this.state === 'running' && (this.pc.connectionState === 'failed' || this.pc.connectionState === 'closed')) {
      this.#handleError('peer connection closed');
    }
  }

  #onTrack(evt) { if (this.conf.onTrack) this.conf.onTrack(evt); }
  #onDataChannel(evt) { if (this.conf.onDataChannel) this.conf.onDataChannel(evt); }
}
