
a.out: Controller.o Localization.o Vision.o BehaviouralPlanner.o LocalPlanner.o VelocityPlanner.o CollisionChecker.o PathOptimizer.o
	g++ main.cpp -std=c++11 `pkg-config --cflags --libs opencv`
	
Controller.o: controller/Controller2D.cpp controller/Controller2D.h
	g++ -c controller/Controller2D.cpp `pkg-config --cflags --libs opencv`

Localization.o: localization/Localization.h
	g++ -c localization/Localization.h -o Localization.o `pkg-config --cflags --libs opencv`

Vision.o: vision/Vision.h
	g++ -c vision/Vision.h -o Vision.o `pkg-config --cflags --libs opencv`

BehaviouralPlanner.o: planner/BehaviouralPlanner.h
	g++ -c planner/BehaviouralPlanner.h -o BehaviouralPlanner.o `pkg-config --cflags --libs opencv`

LocalPlanner.o: planner/LocalPlanner.h
	g++ -c planner/LocalPlanner.h -o LocalPlanner.o `pkg-config --cflags --libs opencv`

VelocityPlanner.o: planner/VelocityPlanner.h
	g++ -c planner/VelocityPlanner.h -o VelocityPlanner.o `pkg-config --cflags --libs opencv`

CollisionChecker.o: planner/CollisionChecker.h
	g++ -c planner/CollisionChecker.h -o CollisionChecker.o `pkg-config --cflags --libs opencv`

PathOptimizer.o: planner/PathOptimizer.h
	g++ -c planner/PathOptimizer.h -o PathOptimizer.o `pkg-config --cflags --libs opencv`

clean:
	rm *.o a.out

