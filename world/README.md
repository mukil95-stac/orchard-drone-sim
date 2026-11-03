# World

This part owns the MuJoCo orchard environment:

- `../models/orchard.xml` is the generated orchard and helipad world.
- `../models/drone.xml` is the standalone drone model.
- `../models/arena_bot.xml` is the separate wheeled-robot world.
- `../models/generate_models.py` regenerates the generated XML files.

Open the world without a controller:

```bash
cd /home/mukil/orchard_drone_sim
python3 world/run_world.py
```

The world viewer only advances MuJoCo physics. It does not arm the drone or
apply controller motor commands.

