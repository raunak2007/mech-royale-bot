# Mech Royale bot. The bot is a single static binary (source in src/, rebuild with ./build.sh).
FROM python:3.13-slim
WORKDIR /bot
COPY . .
RUN chmod 755 /bot/mechbot
ENTRYPOINT ["/bot/mechbot"]
