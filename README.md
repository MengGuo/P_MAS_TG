P-MAS-TG
========

Planner for Multiple Agent System with Temporal Goals 

```
@article{guo2015multi,
  title={Multi-agent plan reconfiguration under local LTL specifications},
  author={Guo, Meng and Dimarogonas, Dimos V},
  journal={The International Journal of Robotics Research},
  volume={34},
  number={2},
  pages={218--235},
  year={2015},
  publisher={SAGE Publications Sage UK: London, England}
}
```

-----
Description
-----
This package contains implementation for plan synthesis algorithms given a finite transition system (as the agent motion model) and a Linear temporal logic formula (as the agent task). It outputs the static plan as a sequence of agent motion and action, required to fulfill the task. 

<p align="center">  
  <img src="https://github.com/MengGuo/P_MAS_TG/blob/master/Intro/figures/collaborate.jpg" width="600"/>
</p>

<p align="center">  
  <img src="https://github.com/MengGuo/P_MAS_TG/blob/master/Intro/figures/nor.png" width="600"/>
</p>

-----
References
-----

**Multi-agent Plan Reconfiguration under Local LTL Specifications**.
Meng Guo and Dimos V. Dimarogonas. International Journal of Robotics Research (IJRR), 34(2): 218-235, Feb 2015. [\[link\]](http://journals.sagepub.com/doi/abs/10.1177/0278364914546174) [\[PDF\]](https://people.kth.se/~mengg/papers/ijrr15.pdf) 

**Task and Motion Coordination for Heterogeneous Multi-agent Systems with Loosely-coupled Local Tasks**.
Meng Guo and Dimos V. Dimarogonas. IEEE Transactions on Automation Science and Engineering (T-ASE), 4(2): 797-808, Apr 2017. [\[link\]](http://ieeexplore.ieee.org/document/7778995/) [\[PDF\]](https://people.kth.se/~mengg/papers/tase17.pdf)


-----
Features
-----
* Allow both normal and co-safe LTL task formulas. 
* Action model can be muted if only motion is concerned.
* Soft specification is optional.
* NetworkX structure for FTS, Buchi and Product automata.
* Static or on-the-fly construction of the product automaton.
* Stand-alone planner.

```python
from P_MAS_TG.ts import MotionFts, ActionModel, MotActModel
from P_MAS_TG.planner import ltl_planner

# construct your motion and action model
#---------
robot_motion = MotionFts(node_dict, symbols, 'your_ws_name')
robot_motion.set_initial(initial_node)
robot_motion.add_un_edges(edge_list, unit_cost = 0.1)
#---------
robot_action = ActionModel(action_dict)
#---------
robot_model = MotActModel(robot_motion, robot_action)

# specify your hard and soft tasks
hard_task = '(([]<> r3) && ([]<> r4))'
soft_task = None

# set planner
robot_planner = ltl_planner(robot_model, hard_task, soft_task)

# synthesis
robot_planner.optimal(10,'static')
```

* Generate `.dat` for MatLAB to load Buchi and product automata model. See [square_world.py](https://github.com/MengGuo/P_MAS_TG/blob/master/Intro/Examples/to_matlab/square_world.py).





----
Debugging
----
* Install python packages like networkx, ply. [*Update to networkx 2.0*]
* Add this package to your PYTHONPATH, to import it in your own project.
* ltlba_32 and ltlba_64 are executable files complied under OS X. For other OS, please follow [ltl2ba/README.txt](https://github.com/MengGuo/P_MAS_TG/blob/master/Install_ltl2ba/README.txt).
* Try [test.py](https://github.com/MengGuo/P_MAS_TG/blob/master/test.py) and other examples in the [Examples](https://github.com/MengGuo/P_MAS_TG/tree/master/Intro/Examples) folder. 
