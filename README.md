# RoboTour2026

Science Olympiad Robot Tour, 2025–26. **1st place, NY States.**

LEGO Technic Hub + [Pybricks](https://pybricks.com/), two drive motors, gyro heading hold.

## Path encoding

Routes are written as one string, one letter per move, so I can change them quickly at the table:

```python
PATH = "IDWWSAQQDWSDQDWAAQWWSSSAWEDQAASDDWAQZAWWDDSAEP"
```

`I` start offset · `W/S` ±1 square · `A/D` 90° turns · `Q/E/Z/C` 90° arcs · `R/Y/F/H` U-turns · `G/L` bottle deposit · `P` final back-up

## Time targeting

Each move's time at base speed is measured in `TIME_ESTIMATES`. Before the run, the code sums the path's expected time, subtracts the fixed per-move overhead (~22 ms), and solves for one speed factor `k` that hits `TARGET_TIME`. Velocities scale by `k` and accelerations by `k²`, so every move keeps the same profile shape.

## Files

- `RotoMain.py`: competition program
- `DiameterFinder.py`: wheel diameter calibration
- `GyroTest.py`: gyro heading-correction calibration
