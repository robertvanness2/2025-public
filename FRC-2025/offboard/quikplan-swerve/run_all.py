from multiprocessing import Process
import os

import run_simple_test
import run_square_test
import run_crazy_test

import run_center_1_piece
import run_left_start_to_back
import run_BL_S_RET_FL
import run_FL_S_RET_FL
import run_FL_S_RET_BL
import run_FL_S_RET_F
import run_left_start_to_front
import run_front_to_right_source


if __name__ == "__main__":
    # Delete old plots and CSVs
    dirname = os.path.dirname(__file__)
    for item in os.listdir(os.path.join(dirname, "plots")):
        if item.endswith(".png"):
            os.remove(os.path.join(dirname, "plots", item))
    for item in os.listdir(os.path.join(dirname, "../../src/main/deploy")):
        if item.endswith(".csv"):
            os.remove(os.path.join(dirname, "../../src/main/deploy", item))

    # Test plans
    Process(target=run_simple_test.plan, args=(True,)).start()
    Process(target=run_square_test.plan, args=(True,)).start()
    Process(target=run_crazy_test.plan, args=(True,)).start()

    # Autos
    Process(target=run_center_1_piece.plan, args=(True,)).start()
    Process(target=run_left_start_to_back.plan, args=(True,)).start()
    Process(target=run_BL_S_RET_FL.plan, args=(True,)).start()
    Process(target=run_FL_S_RET_FL.plan, args=(True,)).start()
    Process(target=run_FL_S_RET_BL.plan, args=(True,)).start()
    Process(target=run_FL_S_RET_F.plan, args=(True,)).start()
    Process(target=run_left_start_to_front.plan, args=(True,)).start()
    Process(target=run_front_to_right_source.plan, args=(True,)).start()
