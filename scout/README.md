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
| v9d | v9b holding 100 m instead of 60 m | 3.98 |
| v9a | v5a plus walk to crates, drones and loose cubes | 4.25 |
| v9c | v9b plus strafe and close in as soon as we open fire | 4.27 |
| v9b | v9a plus go looking for crates and drones when idle | 4.54 |

Each score is the mean of 48 matches; the same code re-run moves by a few tenths.
The self-check build shows no late answers and no crashes in the first 30 s of scored matches.
Pattern so far: the more the bot fights and farms, the higher it scores; hiding, running and rooftops all scored lower.

## What the recordings say

- Top team bots land 80 to 90% of gun shots at 40 to 80 m and about 60% at 80 to 120 m.
- Their lasers hit every time the line is clear, and lasers are close to half of their damage.
- Almost all of them pick up a laser or lightning gun within 5 seconds of the start.
- tigerbyte (near the top of the practice board) creeps at 4 to 5 m/s, which is silent, keeps 100 to
  200 m from everyone, rests under trees, runs and recovers when hurt, and still fights at times.
- Tree crowns block sight beyond 45 m but not shots. Inside 45 m everyone senses everyone.

## Sunday morning: the new scorer (96 matches per run)

Scores under the new scorer are much steadier than the note above says: seven different builds of the
same family landed between 4.96 and 5.13, so a difference of 0.2 between two runs is real.

| branch | change | score |
|---|---|---|
| v10a | fighter that farms and explores | 4.66 |
| f2 | v10a with faster strafing in fights | 4.95 |
| v12b | f2 plus: roam for weapons while the zone allows, look round at spawn, heal before farming | 5.13 |
| v12a, v13a, v14a, v15a, r1, r2 | cover fighting, roof-loot fix, hide from distant lasers, frequent look-round, never fire first with long range | 4.96 to 5.07 |
| v16b | v12b plus: supply drops, loot from death sites, no mortar, tick-exact aim | 5.21 |
| v17a | v16b that never opens fire on a healthy mech | 5.49 |
| v18a | v17a plus break off below half health and heal behind cover | 5.01 |

Diagnostic builds (they play dead when a condition holds, so the drop in score measures the condition):

| branch | plays dead when | score | reading |
|---|---|---|---|
| probe-weapon20 | no second weapon 20 s in | 2.69 | about half of all matches |
| probe-multi | two or more attackers in the first 300 damage | 2.81 | common |
| probe-unseen | half of the first 300 damage came from mechs not seen | 3.96 | about a quarter |
| probe-trade | dealt under 60% of damage taken at 400 taken | 4.84 | rare, or no worse than playing on |
| probe-t45 | always, from 45 s | 0.83 | under one mech is dead by then |
| probe-t105 | always, from 105 s | 3.27 | about three are dead by 110 s |

So in practice matches almost nobody dies in the first 50 seconds, deaths then come about one every 25
seconds, and a bot scoring 5 dies around 150 s. Early fights only cost health. What has paid so far:
farming, drops, not starting fights. What has cost every time it was tried: breaking off to hide.

## Mechanics worked out from the public replays (5,500 matched gun shots)

- Order inside a tick: the torso turns toward the commanded aim, the mech moves, then the weapon fires.
  A shot leaves along the aim commanded in the same decision (if the torso can turn that far in time),
  from where the mech stands after that tick's movement.
- A held gun keeps an exact 0.24 s cadence (ticks 1, 6, 11, 16, 21, 25, 30 ...), so about every other
  shot leaves on the second tick after a decision. Lead by 0.05 s or 0.10 s accordingly.
- Against real movement under fire, a correct straight-line lead with the gun's spread would hit about
  99% at 50 m, 83% at 100 m, 63% at 150 m. Misses beyond that come from targets sidestepping bullets
  they can see.
- Top team bots do 51% of their damage with lasers, 46% with the gun, 2% with lightning, 1% with mortars.
- Damage taken per second by distance to the nearest mech, house bots: under 60 m 66, 60 to 100 m 29,
  100 to 150 m 8, beyond 150 m about 2. With no mech inside 150 m: 2.5.
- On the play site, a mech in the inner quarter of the circle dies about 2.5 times as often as one in the
  band between half and three quarters of the radius.
- Supply drops: in the top-16 matches the same one or two bots take all three (about 4 s after landing
  for the second and third, so they wait at the spot).
