FROM python:3.12-slim AS build
RUN apt-get update && apt-get install -y --no-install-recommends git ca-certificates && rm -rf /var/lib/apt/lists/*
WORKDIR /build
COPY tools/prepare_dependencies.py tools/prepare_dependencies.py
COPY patches patches
COPY requirements.txt requirements.txt
RUN python tools/prepare_dependencies.py --only python-roborock && \
    pip install --no-cache-dir --prefix=/install -r requirements.txt .deps/python-roborock

FROM python:3.12-slim
ENV PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 RR_UI_HOST=0.0.0.0 RR_UI_PORT=8099
COPY --from=build /install /usr/local
RUN useradd --uid 1000 --create-home panel && mkdir /data && chown panel:panel /data
WORKDIR /srv/panel
COPY app/ ./
USER panel
EXPOSE 8099
CMD ["python", "app.py"]
