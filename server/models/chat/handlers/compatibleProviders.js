/**
 * Presets for every server that speaks the OpenAI Chat Completions protocol.
 *
 * **This file is where a new local/compatible provider is added — not a new class.**
 * `OpenAICompatibleHandler` carries the protocol; an entry here carries the quirks.
 *
 * Every field under `capabilities` shapes the OUTGOING request only. Nothing here may
 * influence how a response is parsed: the handler parses defensively and unconditionally,
 * because a server can stop honouring a declared capability without warning (see
 * llama.cpp#20198). Adding a parse-time flag here would re-create the per-provider
 * branching this design exists to remove.
 */

/**
 * Ollama's /v1 has no way to set the context window — the OpenAI schema has no such
 * field and the field is not on Ollama's allowlist, so `num_ctx` sent in the body is
 * dropped in SILENCE (PR ollama/ollama#6137 never landed). A prompt longer than the
 * server default is then truncated with no error, which for the mission planner
 * (~10.8k tokens of system prompt alone) means the model loses its rules and starts
 * inventing waypoints. The context window therefore MUST be configured on the Ollama
 * server itself; this warning fires on every boot because the app cannot verify it.
 */
const OLLAMA_CONTEXT_WARNING =
  'Context window CANNOT be set over the OpenAI-compatible endpoint (num_ctx is ignored silently). ' +
  'Set OLLAMA_CONTEXT_LENGTH on the Ollama server (e.g. OLLAMA_CONTEXT_LENGTH=16384), or bake ' +
  '`PARAMETER num_ctx 16384` into a Modelfile and serve that model. Without it, long prompts are ' +
  'TRUNCATED SILENTLY at the server default and the planner will lose its instructions.';

/**
 * Tool calling depends on the MODEL's chat template, not on a server flag. `--jinja` is
 * default-enabled on recent builds, so the usual failure is subtler: a template with no
 * `tools` branch makes llama-server drop the `tools` array in SILENCE — same prompt_tokens
 * with and without it, and the model answers in prose because it never saw a tool.
 * Vision variants are the classic trap: Qwen2.5-VL's template has no tool support at all,
 * while the text Qwen2.5-Instruct does.
 *
 * Verify before blaming the client:
 *   curl -s localhost:<instance-port>/props | grep -c tool_call   # 0 ⇒ template can't do tools
 */
const LLAMACPP_JINJA_WARNING =
  'Tool calling requires a model whose chat template renders tools (Qwen2.5-Instruct, Hermes 2/3, ' +
  'Llama 3.1/3.3, Mistral Nemo, Functionary v3.x). A template without a `tools` branch — every ' +
  'vision/VL variant, for one — makes the server DROP the tools array silently and reply in prose. ' +
  'Check with: curl -s <server>/props | grep -c tool_call';

export const COMPATIBLE_PROVIDERS = Object.freeze({
  ollama: {
    defaultBaseURL: 'http://localhost:11434/v1',
    defaultModel: 'glm-4.7-flash',
    capabilities: {
      // Listed as UNSUPPORTED in Ollama's OpenAI-compatibility docs.
      toolChoice: false,
      // Not on Ollama's /v1 allowlist either; omitted rather than sent and ignored.
      parallelToolCalls: null,
      // Replaces the native client's `think: false`; Ollama's /v1 does support this one.
      reasoningEffort: 'none',
      temperature: 0.2,
      startupWarning: OLLAMA_CONTEXT_WARNING,
    },
  },

  llamacpp: {
    defaultBaseURL: 'http://localhost:8080/v1',
    defaultModel: 'bartowski/Qwen2.5-7B-Instruct-GGUF:Q4_K_M', //'ggml-org/Qwen2.5-VL-7B-Instruct-GGUF:Q4_K_M',
    capabilities: {
      toolChoice: true,
      // llama.cpp ships multiple tool calls DISABLED by default — the inverse of OpenAI —
      // so the flag has to be sent explicitly to get the behaviour the orchestrator expects.
      parallelToolCalls: true,
      temperature: 0.2,
      startupWarning: LLAMACPP_JINJA_WARNING,
    },
  },

  // Escape hatch for vLLM / LM Studio / OpenRouter / Groq: everything comes from env.
  'openai-compatible': {
    defaultBaseURL: '',
    defaultModel: '',
    capabilities: {
      toolChoice: true,
      temperature: 0.2,
    },
  },
});

/** @returns {boolean} whether `provider` is served by OpenAICompatibleHandler. */
export function isCompatibleProvider(provider) {
  return Object.hasOwn(COMPATIBLE_PROVIDERS, provider);
}

/**
 * Normalizes a host into a Chat Completions base URL.
 * A bare host (`http://localhost:11434`) is accepted so the env var can be written either
 * way, with the `/v1` suffix appended when absent.
 * @param {string} rawURL
 * @returns {string}
 */
export function normalizeBaseURL(rawURL) {
  if (!rawURL) return '';
  const trimmed = rawURL.replace(/\/+$/, '');
  return /\/v\d+$/.test(trimmed) ? trimmed : `${trimmed}/v1`;
}
