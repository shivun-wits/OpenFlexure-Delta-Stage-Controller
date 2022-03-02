# MicroMetal.ino - Documentation

## Serial Command Line Interface

### Low Level Commands

| Command / Code | Description                          | Example                         |
|----------------|--------------------------------------|---------------------------------|
| 1              | Stop All Motors                      | <code> > 1 </code>              |
| 2              | Clock-wise Rotate All Motors         | <code> > 2 </code>              |
| 3              | Counter-Clock-wise Rotate All Motors | <code> > 3 </code>              |
| 4              | Read Encoders                        | <code> > 4 </code>              |
| 5              | Reset Encoders                       | <code> > 5 </code>              |
| 6              | Stop All Motors with holding force   | <code> > 6 </code>              |
| P              | PID commands                         | <code> > PM1.5,5.2,9.22 </code> |

#### PID Commands
To use a PID command start the command with a 'P' and follow with one of the following sub-commands

| PID Sub-command | Description                                                | Example                 |
|-----------------|------------------------------------------------------------|-------------------------|
| Q               | Query Dump PID Info                                        | <code> > PQ </code>     |
| P               | Change Kp = <code>f</code> for motor <code>m</code>        | <code> > PPA100 </code> |
| I               | Change Ki = <code>f</code> for motor <code>m</code>        | <code> > PIA100 </code> |
| D               | Change Kd = <code>f</code> for motor <code>m</code>        | <code> > PDA100 </code> |
| M               | Move motors to absolute pos. (3 comma separated positions) | <code> > PM1.2,1.3,1.5  |
| R               | Move motors to relative pos. (3 comma separated positions) | <code> > PR1.2,1.3,1.5  |
| 0               | Disable PID                                                | <code> > P0 </code>     |
| 1               | Enable PID                                                 | <code> > P1 </code>     |

Note: PID is not enabled upon startup. 
Using low-level command '1' stops PID as well. 