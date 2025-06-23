## Helpful Links
| Description | Link |
| ----------- | ---- |
| Everything you can put in an MJDF file | [XML Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#) |
| Procedurally adding things to an MJDF file | [Tree MJDF Example](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Y4rV2NDh92Ga) |

## Running Log

6/23/25

Figured this is as good a place as any to make notes on my progress in MuJoCo for blueberry cane modeling. 

Today I found a link to a joint option called "springdamper": [Springdamper documentation](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint-springdamper), which led to the rest of the documentation for what MJDF files can actually contain. Which is awesome, because apparently MJDF files natively support springs and dampers on joints! Yay! (It's all on the same page. Just scroll.)

I had already implemented a PD controller with Kp = 295 and Kd = 0.15 on a single inverted pendulum. Just to make sure they were doing the same thing, I ran a 2 second simulation with the same initial conditions and plotted the displacement on the same plot. A timestep of 0.00001 was required to get them to overlap completely; a timestep of 0.0001 had them diverging slightly part of the way through. Both sims used the RK4 integrator. 

![A plot of a damped system coming to rest, with two overlapping lines.](images/PassiveVsPDPend.png)

I think it makes more sense to use the passive components instead of the active ones, especially since it appears that the 3-DOF ball joints are supported. 

MJDFs can be procedurally edited, so I'm going to explore that next. 


