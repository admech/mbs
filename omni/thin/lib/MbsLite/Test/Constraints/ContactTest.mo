within MbsLite.Test.Constraints;

model ContactTest

  import MbsLite.Examples.OmniVehicle.PointContact.RollerPointContactForcesGeneral;

  constant Real[3] forward  = { 1, 0, 0 };
  constant Real[3] vertical = { 0, 1, 0 };
  constant Real[3] userward = { 0, 0, 1 };

  parameter Integer n    = 5;
  parameter Integer nOne = 5;
  parameter Real psi = 0;
  parameter Real wheelHubMass = 0.15;
  parameter Real rollerMass = 0.05;
  parameter Real R = 0.05 "'wheel radius' := distance from wheel axis to the floor";

  parameter Real halfRollerAngle = pi / n;
  parameter Real wheelHubRadius = R * cos(halfRollerAngle);
  parameter Real rollerRadius = R - wheelHubRadius;
  parameter Real rollerLength = 2 * R * sin(halfRollerAngle);
  parameter Real rollerRadiusForMoi = (R - wheelHubRadius) / 2;
  parameter Real[3] wheelR = { 0, R, 0 };

  parameter Real rollerAxialMoi        = rollerMass * (rollerRadiusForMoi^2) / 2;
  parameter Real rollerOrthogonalMoi   = rollerMass / 12 * (3 * rollerRadiusForMoi^2 + rollerLength^2);

  parameter Real    omega0    = 1;
  parameter Real    v0        = omega0 * rollerRadius;
  parameter Real[3] omega0vec = omega0 * (-userward);
  parameter Real[3] r0        = rollerRadius * vertical;
  parameter Real[3] v0vec     = rollerRadius * omega0 * forward;


  NPortsHeavyBody roller
    ( name = "roller"
    , N = 1
    , Gravity = -vertical
    , m = 0.05
    , I = diagonal({ rollerAxialMoi, rollerOrthogonalMoi, rollerOrthogonalMoi })
    , r(start = r0)
    , v(start = v0vec)
    , q(start = QRot(0, { 0, 0, 1 }))
    , omega(start = omega0vec)
    );

  RollerPointContactForcesGeneral contact
    ( name  = "contact"
    , n     = n
    , R     = R
    , psi   = psi
    );

  Base base;

constant Real    expectedForcen = 0.0495225;
constant Real[3] expectedF     = expectedForcen * vertical; // why ?
constant Real[3] expectedFport = expectedF;
constant Real[3] expectedM     = { 0, 0, 0 };
constant Real[3] expectedContactPoint = { 0, 0, 0 };
constant Real[3] expectedContactPointVelocity = { 0, 0, 0 };
initial equation
  AssertReals
    ( expectedContactPoint
    , contact.rB
    , "Contact point"
    );
  AssertReals(r0,        roller.OutPort.r,     "Roller center");
  AssertReals(v0vec,     roller.OutPort.v,     "Roller velocity");
  AssertReals(omega0vec, roller.OutPort.omega, "Roller angular velocity");
  AssertReal (1,         contact.isInContact,  "Roller isInContact");
  AssertReals
    ( Euler
      ( roller.OutPort.r, expectedContactPoint
      , roller.OutPort.v, roller.OutPort.omega
      )
    , contact.vB
    , "Contact point velocity Euler"
    );
  AssertReals
    ( expectedContactPointVelocity
    , contact.vB
    , "Contact point velocity"
    );
  AssertReals
    ( zeros(3)
    , contact.relvt
    , "relvt"
    );
  AssertReal
    ( 0
    , contact.relvn
    , "relvn"
    );
  AssertReal
    ( 0
    , contact.Drelvn
    , "Drelvn"
    );
  AssertReals
    ( zeros(3)
    , contact.Forcet
    , "Forcet"
    );
  AssertReal
    ( 0
    , contact.mu
    , "mu"
    );
  AssertReals
    ( zeros(3)
    , roller.crosses[1]
    , "crosses[1]"
    );
  AssertReal
    ( expectedForcen
    , contact.Forcen
    , "Forcen"
    );
  AssertReals
    ( expectedForcen * vertical
    , roller.a
    , "Roller acceleration"
    );
  AssertReals
    ( expectedF
    , roller.F
    , "Roller force"
    );
  AssertReals
    ( expectedFport
    , roller.InPorts[1].F
    , "Roller force in port"
    );
  AssertReals
    ( expectedM
    , roller.epsilon
    , "Roller angular acceleration"
    );
  AssertReals
    ( expectedM
    , roller.M
    , "Roller torque"
    );
  terminate("passed");

equation

  contact.n1k = roller.T * { 0, 0, 1 };
  contact.rho = normalize(wheelR - roller.r);

  connect(roller.OutPort,    contact.InPortB);
  connect(roller.InPorts[1], contact.OutPortB);
  connect(base.OutPort,      contact.InPortA);

  annotation(experiment
    ( StopTime = 0.1
    , NumberOfIntervals = 100
    , Tolerance = 1e-9
    , Algorithm = "Dassl"
    ));

  when time > 0 then
    Assert
      ( false
      , "should have terminated after checking the solution to the initial problem"
      );
  end when;

end ContactTest;
