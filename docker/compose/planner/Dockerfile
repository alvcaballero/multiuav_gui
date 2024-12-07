FROM  python:3.11.11-bullseye

WORKDIR /usr/src/app

RUN apt-get update && apt-get install -y \
libdbus-1-3 \
libxkbcommon-x11-0 \
libxcb-xinerama0 \
libegl1-mesa \
libfontconfig1 \
libxcomposite1 \
libxi6 \
libgl1 \
libglib2.0-0 \
&& rm -rf /var/lib/apt/lists/*

COPY requirements.txt ./
RUN pip install  --no-cache-dir -r requirements.txt

COPY . .

#CMD ["/bin/sh", "-c", "./usr/src/app/server_linux.sh" ]
#CMD [ "python", "-m" , "flask","--app ./api" ,"run", "--host=0.0.0.0 ","--port=8004"]
CMD ["python", "-m", "flask", "--app", "./api", "run", "--host=0.0.0.0", "--port=8004"]
