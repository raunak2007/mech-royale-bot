# scotty-edge: a Mech Royale bot (mechfight/1)

Plays the edge of the circle: moves inside the next zone early, parks in boxed-in spots away from
the middle, sidesteps bullets it can see, breaks line of sight when shot at, and fights when it has
the better of a duel or nowhere to run.

- `mechbot` is the bot: one static linux/amd64 binary, no runtime dependencies.
- `params.txt` holds the tunables it reads at startup.
- `src/` is the source (Rust, standard library only). `./build.sh` rebuilds `mechbot`.
- `src/sim.rs` and `src/bin/sim.rs` are a local simulator used for testing. They are not part of
  the bot and its "house" opponents are a guess, so its scores do not predict leaderboard scores.

Check it against the kit (from the kit folder, with this repo at ../repo):

    python3 check/check.py -- ../repo/mechbot
    docker build --platform linux/amd64 -t my-bot ../repo
    python3 check/check.py --map-dir /arena -- docker run --rm -i --network none --cpus 1 --memory 512m \
        --read-only --tmpfs /tmp:rw,size=64m --user 65534:65534 -v "$PWD/arena:/arena:ro" my-bot
