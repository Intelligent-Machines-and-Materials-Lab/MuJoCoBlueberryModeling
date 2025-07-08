## Helpful Links
| Description | Link |
| ----------- | ---- |
| Everything you can put in an MJDF file | [XML Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#) |
| Procedurally adding things to an MJDF file | [Tree MJDF Example](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Y4rV2NDh92Ga) |

## Running Log

7/7/25

Spent the day trying different parameters for solimp and solref in an attempt to minimize the difference between the napkin math and the calculated contact force. Attempts are documented here: [Contact definition experiments](https://docs.google.com/spreadsheets/d/1iFUkzn5xVksr9jLl0r_p5st1Fuz1WwpFBKT4AZi-6Zg/edit?usp=sharing) 

Since the branch is very thin, a very quick transition to stiffness was necessary with the solimp values, but a longer time constant could be used on the spring-damper system for the contact (first value of solref). The best values I achieved for the system, over the 45 second simulation, were a napkin math estimate of 12.9N and a measured simulation contact force of 13.55N. About half a newton of error is honestly better than I thought I was going to achieve with this. 

I did find out that the branch was "phasing through" the crossbar in the earlier experiments, which is what was causing the REALLY drastic oscillations seen in the force plots -- the branch was getting stuck between the crossbar and the pushrod. The force plots are much smoother after taking out the "fork" of the forked probe. A thick "probe" geometry should be used if we ever add the forked geometry back in. 

For posterity, the best values: 

```
solref="0.125 1" solimp=".95 1 0.0025"
```

7/3/25

My #1 suspect for the disparity between the napkin math and the force output is the contact forces/stabilization that MuJoCo has to do behind the scenes. I did mess with `solref` and `solimp`, but neither direction makes the forces any closer to the napkin math outputs. 

So my next idea was to make it a closed loop system -- attach the probe to a wall, effectively, and make a hinge joint connecting the probe to the branch, and push on it. However, this introduces constraint stabilization instead, and if it's not perfectly lined up, introduces its own problems, like huge jumps in force/position at the beginning. I've bashed my head long enough against it in the current state to believe it's not going to work. Here's the next ideas I have: 

- a closed kinematic loop with a joint at the wall, a slider, a joint at the connection, and another joint at the base (4 joints total)
- swapping the velocity controller for a position controller so that it STOPS and we the forces can stabilize
- switching to applying a force trajectory at a point instead of having a second body at all. That way we avoid any weird constraints or contacts or any of that. 

7/2/25

mujoco.mj_contactForce() reports the contact force in the "contact frame", which is a frame defined at the point of contact, not in the world frame. 

When we do some quick napkin math:

```
napkin_math_force = (295*branch_max_disp)/PROBE_HEIGHT
```

and compare it to the actual force we're measuring at the contact point, we get (ignore the negative sign):

```
Final branch displacement: 0.019 rad
Napkin math force: 7.553 N
Final contact x force: -11.287 N
```

6/25/25

Trying to figure out how to get the probe to follow a velocity of 1mm/s, because it really doesn't want to do that. See images/plots for different combinations of dt and kv I've attempted to get there (though I've gitignore'd them because it's clutter). It seems like there's a minimum dt to get the simulation to even register contact between the two objects. Then there's a minimum kv to get the probe to push hard enough on the branch to move it, but also a maximum, else it will just go too fast. So there's this little window where it will push consistently. The best I've found so far is around dt = .00001 and kv=34000. It doesn't follow exactly, but it at least pushes at a consistent pace. We can compare force values when the branch has been displaced by a certain amount, rather than at a specific time, if we need to. 

![A plot of three increasing lines, representing the ideal movement of the probe, the actual movement of the probe, and the actual displacement of the branch.](images/joint_positions_kv34000_dt1e-05.png)

It has a rather rough force plot as the branch and probe establish contact, but it smooths out over time to a very reasonable line. 

![A force plot, with a strong oscillation at the beginning, but then a steadily increasing line afterwards](images/x_force_kv34000_dt1e-05.png)

Upon going all the way to 45 seconds (which takes 14 minutes of real time to simulate), it only gets to 30mm. 

6/24/25

MuJoCo has a "position" actuator type that is kind of like a servo motor. My first plan is to run that on the probe and see if we can get it to push on the cane. 

6/23/25

Figured this is as good a place as any to make notes on my progress in MuJoCo for blueberry cane modeling. 

Today I found a link to a joint option called "springdamper": [Springdamper documentation](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint-springdamper), which led to the rest of the documentation for what MJDF files can actually contain. Which is awesome, because apparently MJDF files natively support springs and dampers on joints! Yay! (It's all on the same page. Just scroll.)

I had already implemented a PD controller with Kp = 295 and Kd = 0.15 on a single inverted pendulum. Just to make sure they were doing the same thing, I ran a 2 second simulation with the same initial conditions and plotted the displacement on the same plot. A timestep of 0.00001 was required to get them to overlap completely; a timestep of 0.0001 had them diverging slightly part of the way through. Both sims used the RK4 integrator. 

![A plot of a damped system coming to rest, with two overlapping lines.](images/PassiveVsPDPend.png)

I think it makes more sense to use the passive components instead of the active ones, especially since it appears that the 3-DOF ball joints are supported. 

MJDFs can be procedurally edited, so I'm going to explore that next. 


