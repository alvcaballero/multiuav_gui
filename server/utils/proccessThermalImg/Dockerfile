# ---- builder: git, pip and the SDK zip live here and never reach the final image ----
FROM python:3.9-slim AS builder

WORKDIR /build

RUN apt-get update && \
    apt-get install -y --no-install-recommends git && \
    rm -rf /var/lib/apt/lists/*

COPY requirements.txt ./
RUN python -m venv /opt/venv && \
    /opt/venv/bin/pip install --no-cache-dir -r requirements.txt && \
    # thermal_base pulls the full opencv-python (needs libGL/mesa/llvm). Both wheels share cv2/,
    # so remove both and put back only the headless one.
    /opt/venv/bin/pip uninstall -y opencv-python opencv-python-headless && \
    /opt/venv/bin/pip install --no-cache-dir opencv-python-headless && \
    /opt/venv/bin/pip uninstall -y pip setuptools wheel

# Pre-download DJI Thermal SDK so the container works offline.
# thermal_base only uses linux/release_x64 on a 64-bit interpreter: drop the rest.
RUN /opt/venv/bin/python -c "\
import requests, zipfile, io; \
r = requests.get('https://dtpl-ai-public.s3.ap-south-1.amazonaws.com/Thermal_Image_Analysis/DJI_SDK/dji_executables.zip'); \
zipfile.ZipFile(io.BytesIO(r.content)).extractall('sdk'); \
" && \
    mkdir -p /out/dji_executables/linux && \
    mv sdk/dji_executables/linux/release_x64 /out/dji_executables/linux/ && \
    chmod +x /out/dji_executables/linux/release_x64/dji_irp

# ---- runtime ----
FROM python:3.9-slim

WORKDIR /usr/src/app

RUN apt-get update && \
    apt-get install -y --no-install-recommends libglib2.0-0 libimage-exiftool-perl libgomp1 && \
    rm -rf /var/lib/apt/lists/*

COPY --from=builder /opt/venv /opt/venv
COPY --from=builder /out/dji_executables ./dji_executables
COPY ./processThermalGen.py .

ENV PATH="/opt/venv/bin:$PATH"

RUN exiftool -ver

ENTRYPOINT ["python", "./processThermalGen.py"]
