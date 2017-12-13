#include <OpenSim/OpenSim.h>
using namespace SimTK;
using namespace OpenSim;
int main() {
	Model model;
	model.setName("bicep_curl");
	model.setUseVisualizer(true);

	// Create two links, each with a mass of 1 kg, center of mass at the body's
	// origin, and moments and products of inertia of zero.
	OpenSim::Body * humerus = new OpenSim::Body("humerus", 1, Vec3(0), Inertia(0));
	OpenSim::Body * radius = new OpenSim::Body("radius", 1, Vec3(0), Inertia(0));
	// add exoskeleton ======================================================================
	OpenSim::Body * ex_humerus = new OpenSim::Body("ex_humerus", 1, Vec3(0), Inertia(0));
	OpenSim::Body * ex_radius = new OpenSim::Body("ex_radius", 1, Vec3(0), Inertia(0));

	// Connect the bodies with pin joints. Assume each body is 1m long.
	PinJoint * shoulder = new PinJoint("shoulder",
		model.getGround(), Vec3(0), Vec3(0),
		*humerus, Vec3(0, 1, 0), Vec3(0));
	PinJoint * elbow = new PinJoint("elbow",
		*humerus, Vec3(0), Vec3(0),
		*radius, Vec3(0, 1, 0), Vec3(0));

	// Connect the exoskeleton bodies========================================================
	PinJoint * ex_shoulder = new PinJoint("ex_shoulder",
		model.getGround(), Vec3(0, 0, 0.25), Vec3(0),
		*ex_humerus, Vec3(0, 1, 0), Vec3(0));
	PinJoint * ex_elbow = new PinJoint("ex_elbow",
		*ex_humerus, Vec3(0), Vec3(0),
		*ex_radius, Vec3(0, 1, 0), Vec3(0));

	// Joint range
	double angleRange[2] = { 0, Pi / 4 };
	//elbow->updCoordinate(PinJoint::Coord::RotationZ).setRange(angleRange);
	ex_elbow->updCoordinate(PinJoint::Coord::RotationZ).setRange(angleRange);

	// Add a muscle that flexes the elbow.
	Millard2012EquilibriumMuscle * biceps = new Millard2012EquilibriumMuscle("biceps",
		400, 0.6, 0.55, 0);
	biceps->addNewPathPoint("origin", *humerus, Vec3(0, 0.8, 0));
	biceps->addNewPathPoint("insertion", *radius, Vec3(0, 0.7, 0));

	// Add a linear actuator==============================

	// Add a controller for muscle
	PrescribedController * brain = new PrescribedController();
	brain->addActuator(*biceps);
	brain->prescribeControlForActuator("biceps", new StepFunction(0.5, 3, 0.3, 1));

	// Add a controller for actuator==========================

	// Add components to the model.
	model.addBody(humerus);
	model.addBody(radius);
	model.addJoint(shoulder);
	model.addJoint(elbow);
	model.addForce(biceps);
	model.addController(brain);

	//=======================================================================================
	// Add Bushing force between body and ex_body.
	BushingForce * bushingForce_1 = new BushingForce("bushingForce_1", 
		"humerus", Vec3(0, 0.5, 0), Vec3(0),
		"ex_humerus", Vec3(0, 0.5, 0), Vec3(0),
		Vec3(1.0), Vec3(1.0), Vec3(10.0), Vec3(10.0));
	BushingForce * bushingForce_2 = new BushingForce("bushingForce_2",
		// body1Name, location in body1, orientation in body1
		"radius", Vec3(0, 0.5, 0), Vec3(0),
		// body2Name, location in body2, orientation in body2
		"ex_radius", Vec3(0, 0.5, 0), Vec3(0),
		//transform stiffness, orientation stiffness, transform damping, orientation damping
		Vec3(1000.0), Vec3(100.0), Vec3(100.0, 100.0, 0.0), Vec3(0));
	//=======================================================================================

	// Add ex components to model.
	model.addBody(ex_humerus);
	model.addBody(ex_radius);
	model.addJoint(ex_shoulder);
	model.addJoint(ex_elbow);
	model.addForce(bushingForce_1);
	model.addForce(bushingForce_2);

	// Add a console reporter to print the muscle fiber force and elbow angle.
	ConsoleReporter * reporter = new ConsoleReporter();
	reporter->set_report_time_interval(1.0);
	reporter->addToReport(biceps->getOutput("fiber_force"), "fiber_force");
	reporter->addToReport(elbow->getCoordinate(PinJoint::Coord::RotationZ).getOutput("value"), "elbow_angle");
	reporter->addToReport(ex_elbow->getCoordinate(PinJoint::Coord::RotationZ).getOutput("value"), "ex_elbow_angle");
	model.addComponent(reporter);

	// Add display geometry.
	Ellipsoid bodyGeometry_1(0.1, 0.5, 0.1);
	bodyGeometry_1.setColor(Gray);
	Ellipsoid bodyGeometry_2(0.05, 0.5, 0.05);
	bodyGeometry_2.setColor(Gray);

	// Attach an ellipsoid to a frame located at the center of each body.
	PhysicalOffsetFrame * humerusCenter = new PhysicalOffsetFrame("humerusCenter", "humerus", Transform(Vec3(0, 0.5, 0)));
	humerus->addComponent(humerusCenter);
	humerusCenter->attachGeometry(bodyGeometry_1.clone());
	PhysicalOffsetFrame * radiusCenter = new PhysicalOffsetFrame("radiusCenter", "radius", Transform(Vec3(0, 0.5, 0)));
	radius->addComponent(radiusCenter);
	radiusCenter->attachGeometry(bodyGeometry_1.clone());

	// Attach an ellipsoid to a frame located at the center of each ex_body
	PhysicalOffsetFrame * ex_humerusCenter = new PhysicalOffsetFrame("ex_humerusCenter", "ex_humerus", Transform(Vec3(0, 0.5, 0)));
	ex_humerus->addComponent(ex_humerusCenter);
	ex_humerusCenter->attachGeometry(bodyGeometry_2.clone());
	PhysicalOffsetFrame * ex_radiusCenter = new PhysicalOffsetFrame("ex_radiusCenter", "ex_radius", Transform(Vec3(0, 0.5, 0)));
	ex_radius->addComponent(ex_radiusCenter);
	ex_radiusCenter->attachGeometry(bodyGeometry_2.clone());

	// Configure the model.
	State & state = model.initSystem();
	// Fix the shoulder at its default angle and begin with the elbow flexed.
	shoulder->getCoordinate().setLocked(state, true);
	elbow->getCoordinate().setValue(state, 0.5 * Pi);
	// Fix exo default
	ex_shoulder->getCoordinate().setLocked(state, true);
	ex_elbow->getCoordinate().setValue(state, 0.5 * Pi);
	model.equilibrateMuscles(state);

	// Configure the visualizer.
	model.updMatterSubsystem().setShowDefaultGeometry(true);
	Visualizer & viz = model.updVisualizer().updSimbodyVisualizer();
	viz.setBackgroundType(viz.SolidColor);
	viz.setBackgroundColor(White);

	// Simulate.
	simulate(model, state, 10.0);

	return 0;
};
