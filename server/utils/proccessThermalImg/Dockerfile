# Use an official Python runtime as a parent image
FROM python:3.9-slim

# Set the working directory in the container
WORKDIR /usr/src/app

# Copy the current directory contents into the container at /usr/src/app

COPY requirements.txt ./


# Install any needed packages specified in requirements.txt
RUN apt-get update && \
    apt-get install -y git libgl1 libglib2.0-0 libimage-exiftool-perl libgomp1 && \
    rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir -r requirements.txt

COPY ./processThermalGen.py .

# Pre-download DJI Thermal SDK so the container works offline
RUN python -c "\
import requests, zipfile, io; \
from pathlib import Path; \
r = requests.get('https://dtpl-ai-public.s3.ap-south-1.amazonaws.com/Thermal_Image_Analysis/DJI_SDK/dji_executables.zip', stream=True); \
zipfile.ZipFile(io.BytesIO(r.content)).extractall(); \
" && \
    chmod +x dji_executables/linux/release_x64/dji_irp && \
    chmod +x dji_executables/linux/release_x86/dji_irp

RUN exiftool -ver

ENTRYPOINT ["python", "./processThermalGen.py"]