# Companion server image (simulation / bench use; on the robot use the systemd unit).
#   docker build -f pi/Dockerfile -t pipebot-pi .
#   docker run -p 8000:8000 -e PIPEBOT_LINK=tcp://host.docker.internal:7000 pipebot-pi
FROM python:3.12-slim
RUN apt-get update && apt-get install -y --no-install-recommends libavformat-dev libavcodec-dev libswscale-dev libsrtp2-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/pipebot
COPY pi/requirements.txt pi/requirements.txt
RUN pip install --no-cache-dir -r pi/requirements.txt
COPY pi pi
COPY app/dist app/dist
ENV PIPEBOT_CAMERA=synthetic PIPEBOT_STATIC=/opt/pipebot/app/dist
EXPOSE 8000
CMD ["uvicorn", "pi.app.main:app", "--host", "0.0.0.0", "--port", "8000"]
