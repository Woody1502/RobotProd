FROM python:3.12-slim

WORKDIR /app
COPY emulator.py .

ENV EMU_PORT=81
CMD ["python3", "-u", "emulator.py"]
