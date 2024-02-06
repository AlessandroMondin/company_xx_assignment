# Use the official Python3.10 runtime as parent image
FROM python:3.10

# Set /usr/src/app as working directory in the container
WORKDIR /usr/src/app

# Copy the current directory into the container at /usr/src/app
COPY . .

# Install packages specified in requirements.txt (none for now)
RUN pip install --no-cache-dir -r requirements.txt

# Build and install the driving_analysis Python package
RUN python setup.py install

# Make port 80 available to the world outside this container
EXPOSE 80

# Set the entrypoint to script responsible to process the json input data
ENTRYPOINT ["python", "./main.py"]
# Set default arguments for CMD
CMD ["--help"]
