#include <OpenSim/OpenSim.h>
using namespace SimTK;
using namespace OpenSim;
int main() {
	Model model;
	model.setUseVisualizer(true);

	OpenSim::Body * humerus = new OpenSim::Body("humerus", 1, Vec3(0), Inertia(0));
	OpenSim::Body * radius = new OpenSim::Body("radius", 1, Vec3(0), Inertia(0));

	PinJoint * shoulder = new PinJoint("shoulder",
		model.getGround(), Vec3(0), Vec3(0),
		*humerus, Vec3(0, 1, 0), Vec3(0));
	PinJoint * elbow = new PinJoint("elbow",
		*humerus, Vec3(0), Vec3(0),
		*radius, Vec3(0, 1, 0), Vec3(0));

	Millard2012EquilibriumMuscle * biceps = new Millard2012EquilibriumMuscle("biceps",
		200, 0.6, 0.55, 0);
	biceps->addNewPathPoint("origin", *humerus, Vec3(0, 0.8, 0));
	biceps->addNewPathPoint("insertion", *radius, Vec3(0, 0.7, 0));

	PrescribedController * brain = new PrescribedController();
	brain->addActuator(*biceps);
	brain->prescribeControlForActuator("biceps", new StepFunction(0.5, 3, 0.3, 1));

	model.addBody(humerus);
	model.addBody(radius);
	model.addJoint(shoulder);
	model.addJoint(elbow);
	model.addForce(biceps);
	model.addController(brain);

	ConsoleReporter * reporter = new ConsoleReporter();
	reporter->set_report_time_interval(1.0);
	reporter->addToReport(biceps->getOutput("fiber_force"));
	reporter->addToReport(elbow->getCoordinate(PinJoint::Coord::RotationZ).getOutput("value"));
	model.addComponent(reporter);

	//Ellipsoid * bodyGeometry(0.1, 0.5, 0.1);
	
	//bodyGeometry.setColor(Gray);
	Ellipsoid * bodyGeometry = new Ellipsoid(0.1, 0.5, 0.1);
	bodyGeometry->setColor(Gray);
	PhysicalOffsetFrame * humerusCenter = new PhysicalOffsetFrame("humerusCenter", "humerus", Transform(Vec3(0, 0.5, 0)));
	humerus->addComponent(humerusCenter);
	humerusCenter->attachGeometry(bodyGeometry->clone());
	PhysicalOffsetFrame * radiusCenter = new PhysicalOffsetFrame("radiusCenter", "raiuds", Transform(Vec3(0, 0.5, 0)));
	radius->addComponent(radiusCenter);
	radiusCenter->attachGeometry(bodyGeometry->clone());

	State & state = model.initSystem();
	shoulder->getCoordinate().setLocked(state, true);
	elbow->getCoordinate().setValue(state, 0.5 * Pi);
	model.equilibrateMuscles(state);

	model.updMatterSubsystem().setShowDefaultGeometry(true);
	Visualizer & viz = model.updVisualizer().updSimbodyVisualizer();
	viz.setBackgroundColor(White);

	RungeKuttaMersonIntegrator integrator(model.getSystem());
	Manager manager(model, integrator);
	manager.setInitialTime(0);
	manager.setFinalTime(10.0);
	manager.integrate(state);

	return 0;
}