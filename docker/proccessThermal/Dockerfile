# Use an official Python runtime as a parent image
FROM python:3.9-slim

# Set the working directory in the container
WORKDIR /usr/src/app

# Copy the current directory contents into the container at /usr/src/app

COPY requirements.txt ./


# Install any needed packages specified in requirements.txt
RUN apt-get update 
RUN apt-get install -y git libgl1 libglib2.0-0 libimage-exiftool-perl
RUN rm -rf /var/lib/apt/lists/*
RUN pip install --no-cache-dir -r requirements.txt
COPY ./processThermalGen.py .

COPY ./DJI_THRM.jpg .

RUN exiftool -ver

#RUN  python processThemalimages.py -i /usr/src/app/DJI_THRM.jpg -o /usr/src/app/DJI_THRM_out.jpg

# Run processThemalimages.py when the container launches
#ENTRYPOINT ["python", "./processThemalimages.py"]
CMD ["bash"]