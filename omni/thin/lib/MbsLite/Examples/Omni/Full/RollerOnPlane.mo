within MbsLite.Examples.Omni.Full;

model RollerOnPlane "A roller that is constrained as if it were attached to a wheel, contacting with the floor."
  import MbsLite.Examples.Omni.PlaneContact;

  constant  Real[3] rollerAxisLocal  = forward;
  constant  Real[3] wheelAxis        = userward;

  parameter String  name   = "OmniWheelOnPlaneFree";

  parameter Real[3]  Gravity = fill(inf, 3);
  parameter Integer  nActual = -Integer_inf;
  parameter Real[3]  r0      = fill(inf, 3);
  parameter Real[4]  q0      = fill(inf, 4);
  parameter Params         params;
  parameter Initials       initials;
  parameter FrictionParams frictionParams;

  parameter Real[3, 3]   T0  = QToT(q0);

  parameter Real     RollerAngle            = 0 "Angles between downward vertical { 0, -1, 0 } and roller center radius vectors";
  parameter Real[4]  RollerQRel             = QRot(RollerAngle, wheelAxis);
  parameter Real[4]  RollerQAbs             = QMult(q0, RollerQRel);

  parameter Real[3]  RollerVerticalInWheelCoords       = QToT(RollerQRel) * vertical;
  parameter Real[3]  RollerCenterDirection             = -RollerVerticalInWheelCoords;
  parameter Real[3]  RollerAxisDirectionInWheelCoords  = QToT(RollerQRel) * rollerAxisLocal;
  parameter Real[3]  RollerCenterInWheelCoords         = params.wheelHubRadius * RollerCenterDirection;

  Base base;

  NPortsHeavyBody roller
      ( final name = "roller"
      , final m = params.rollerMass
      , final I = diagonal({ params.rollerAxialMoi, params.rollerOrthogonalMoi, params.rollerOrthogonalMoi })
      , final N = 1
      , final Gravity = Gravity
      , final r(start = r0 + T0 * RollerCenterInWheelCoords)
      , final v(start = initials.vVec + cross(initials.omegaVec, T0 * RollerCenterInWheelCoords))
      , final q(start = RollerQAbs)
      , final omega(start = transpose(QToT(RollerQAbs)) * initials.omegaVec)
      );

  PlaneContact contact
    ( name = "contact"
    , params                        = params
    , isInContactInitially          = true
    , frictionParams                = frictionParams
    );

initial algorithm
  AssertInitialized (name, q0,      "q0");
  AssertInitialized (name, r0,      "r0");
  AssertInitialized (name, Gravity, "Gravity");

equation

  connect(contact.InPortA,    base.OutPort);
  connect(contact.InPortB,    roller.OutPort);
  connect(contact.OutPortB,   roller.InPorts[1]);

  assert(noEvent(contact.isInContactNormal), "Roller would have left contact already (now it just tipped over).");

end RollerOnPlane;

