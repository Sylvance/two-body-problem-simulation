FROM python:3.6-slim

ENV KUNS_VERSION 0.6.7
RUN pip3 install numpy matplotlib pandas

COPY sim3d_animated.py src/sim3d_animated.py

CMD ["python3", "src/sim3d_animated.py"]
