import prime_bullet as pb
import numpy as np
import signal

class AppState:
    should_stop = False


def main():

    app = AppState()

    def cb_interrupt(*args):
        app.should_stop = True

    signal.signal(signal.SIGINT, cb_interrupt)

    sim = pb.Simulator(real_time=True)
    sim.init('gui')

    ground = sim.create_box((10, 10, 1),
                            pb.Transform.from_xyz(0, 0, -0.5),
                            mass=0, color=(1, 1, 1, 1))
    boxes = []
    for y in range(5):
        for x in range(5):
            box = sim.create_box([1 - x*0.1]*3,
                                pb.Transform.from_xyz(0, (y - 2) * 1.2, x + 0.5),
                                color=(1, y / 4, 0, 1)) # type: pb.RigidBody
            box.dynamics_info.mass = x * 0.5 + 1
            box.dynamics_info.restitution = ((y + 1) / 5 - 1e-3) 
            print(box.dynamics_info.lateral_friction)
            box.dynamics_info.lateral_friction  = 0.82
            boxes.append(box)

    while not app.should_stop:
        sim.update()


if __name__ == '__main__':
    main()