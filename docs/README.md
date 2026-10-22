# Docs for agents

These files describe the working system as of the hallway benchmark: a simulated Unitree G1 EDU stands, walks, is driven from the Foxglove web app or walks a closed hallway to its end wall on its own, and each run is recorded as MCAP.

Read in this order:

| File | What it settles |
| --- | --- |
| [overview.md](overview.md) | What the trial is, what it is not, and the features that work |
| [architecture.md](architecture.md) | The three processes, how they talk, and which file owns what |
| [decisions.md](decisions.md) | Choices already made, and the failure each one prevents |
| [interfaces.md](interfaces.md) | DDS topics, Foxglove topics and services, environment, exit codes |
| [operations.md](operations.md) | How to run, check, and avoid the usual false alarms |
| [roadmap.md](roadmap.md) | What is next, and in what order |
| [health.md](health.md) | Health states, the incident event MCAP, and how that file becomes a Foxglove event |

`README.md` at the repository root is for a person standing the robot up. Prefer these files when the task is to change code.

When a doc and the code disagree, the code is right. Update the doc in the same change.
