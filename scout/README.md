# Scouting notes (Sat night, Nov 21)

## How to watch real matches in full

The Watch page's broadcast is a public WebSocket (`wss://watch.joinbuilders.org/live`) that streams
every mech's position, aim, HP, cubes and weapons at 10 Hz, plus every shot, hit, pickup and death.
On connecting it replays the current match from its start.

1. Open any page on `https://watch.joinbuilders.org` (for example `/show/show.js`, which is light).
2. Paste `recorder.js` into the browser console. It keeps the last 12 matches in `window.REC.all`
   and a one-line summary of each finished match in `localStorage["REC_SUMM"]`.
3. Two kinds of match are shown in turn: "Teams' bots · Showdown" (the current top 16 against each
   other, which is the format of the final) and "Real players" (one human against 15 house bots).

The house bots on the play site are softer than the ones in scored practice matches: their lasers
hit about a quarter of the time, while the house bot in the kit's own recording lands 4 of 4.

## What scored, in order

| branch | idea | score |
|---|---|---|
| idle | statue | 0.02 |
| v8a | camp on a rooftop | 1.88 |
| v3a | run from anyone who sees us | 2.38 |
| v6b | stand inside a tree crown (ellipsoid crowns) | 2.42 |
| v6a | stand inside a tree crown (cylinder crowns) | 2.88 |
| quiet | hide, never fire first | 3.08 |
| main (v1) | hide, fight back, finish weak targets | 3.31 |
| v7a | fighter that breaks off at 55% HP to recover | 3.38 |
| probe-late | v4a plus a self-check for late answers | 3.83 |
| v5a | v4a plus run for a laser first | 3.90 |
| v4a | take every duel, hold 60 m | 3.94 |

Each score is the mean of 48 matches; the same code re-run moves by a few tenths.
The self-check build proves the bot never answers late and never crashes in scored matches.

## What the recordings say

- Top team bots land 80 to 90% of gun shots at 40 to 80 m and about 60% at 80 to 120 m.
- Their lasers hit every time the line is clear, and lasers are close to half of their damage.
- Almost all of them pick up a laser or lightning gun within 5 seconds of the start.
- tigerbyte (near the top of the practice board) creeps at 4 to 5 m/s, which is silent, keeps 100 to
  200 m from everyone, rests under trees, runs and recovers when hurt, and still fights at times.
- Tree crowns block sight beyond 45 m but not shots. Inside 45 m everyone senses everyone.
