within MbsLite.Examples.OmniVehicle.Full;

model RollerOnPlaneExplicit "A roller that is constrained as if it were attached to a wheel, contacting with the floor."
  import MbsLite.Examples.OmniVehicle.Contact.PlaneContactOldSchool;

  constant  Real[3] rollerAxisLocal  = forward;
  constant  Real[3] wheelAxis        = userward;

  parameter String  name   = "OmniWheelOnPlaneFree";

  parameter Real[3]  Gravity = fill(inf, 3);
  parameter Integer  nActual = -Integer_inf;
  parameter Real[3]  r0      = fill(inf, 3);
  parameter Real[4]  q0      = fill(inf, 4);
  parameter Params   params;
  parameter Initials initials;
  parameter Real     coefficientOfFriction;
  parameter Real     viscousFrictionVelocityBound;

  parameter Real[3, 3]   T0  = QToT(q0);

  parameter Real     RollerAngle            = 0 "Angles between downward vertical { 0, -1, 0 } and roller center radius vectors";
  parameter Real[4]  RollerQRel             = QRot(RollerAngle, wheelAxis);
  parameter Real[4]  RollerQAbs             = QMult(q0, RollerQRel);

  parameter Real[3]  RollerVerticalInWheelCoords       = QToT(RollerQRel) * vertical;
  parameter Real[3]  RollerCenterDirection             = -RollerVerticalInWheelCoords;
  parameter Real[3]  RollerAxisDirectionInWheelCoords  = QToT(RollerQRel) * rollerAxisLocal;
  parameter Real[3]  RollerCenterInWheelCoords         = params.wheelHubRadius * RollerCenterDirection;


  parameter Real        m = params.rollerMass;
  parameter Real[3, 3]  I = diagonal({ params.rollerAxialMoi, params.rollerOrthogonalMoi, params.rollerOrthogonalMoi });

  Real[3]  rollerCenterCoordsGlobal         (start = r0 + T0 * RollerCenterInWheelCoords);
  Real[3]  rollerCenterVelocityGlobal       (start = initials.vVec + cross(initials.omegaVec, T0 * RollerCenterInWheelCoords));
  Real[3]  rollerCenterAccelerationGlobal;
  Real[3]  rollerAngularVelocityLocal       (start = transpose(QToT(RollerQAbs)) * initials.omegaVec);
  Real[3]  rollerAngularAccelerationLocal;
  Real[3]  friction;
  Real     normalReaction;
  Real[3]  wheelVerticalGlobal              (start = vertical);
  Real[3]  wheelAngularVelocityGlobal;
  Real[3]  contactPointCoordsRelGlobal;
  Real[3]  contactPointVelocityAbsGlobal;

  Real[4]  rollerQuaternion                 (start = q0);
  Real[3, 3] T;
  Real     contactPointVelocityAbsGlobalNorm;
  Real[3]  rollerAngularVelocityGlobal;
  Real[3]  rollerVerticalGlobal;

initial algorithm
  AssertInitialized (name, q0,      "q0");
  AssertInitialized (name, r0,      "r0");
  AssertInitialized (name, Gravity, "Gravity");

equation
  
  // rigid body dynamics

  m * rollerCenterAccelerationGlobal = m * Gravity + normalReaction * vertical + friction;
  der(rollerCenterVelocityGlobal) = rollerCenterAccelerationGlobal;
  der(rollerCenterCoordsGlobal) = rollerCenterVelocityGlobal;

  T = QToT(rollerQuaternion);
  I * rollerAngularAccelerationLocal + cross(rollerAngularVelocityLocal, I * rollerAngularVelocityLocal) = T * cross(contactPointCoordsRelGlobal, friction);
  der(rollerAngularVelocityLocal) = rollerAngularAccelerationLocal;
  der(rollerQuaternion) = 0.5 * QMult(rollerQuaternion, { 0, rollerAngularVelocityLocal[1], rollerAngularVelocityLocal[2], rollerAngularVelocityLocal[3] });

  // Signorini non-fall-through

  rollerCenterAccelerationGlobal[2] = 0;

  // friction

  friction = -coefficientOfFriction * contactPointVelocityAbsGlobal * normalReaction
           / (if noEvent(contactPointVelocityAbsGlobalNorm < viscousFrictionVelocityBound) then viscousFrictionVelocityBound else contactPointVelocityAbsGlobalNorm);

  // contact tracking

  rollerAngularVelocityGlobal        = T * rollerAngularVelocityLocal;
  wheelAngularVelocityGlobal         = T * (rollerAngularVelocityLocal - { rollerAngularVelocityLocal[1], 0, 0 });
  der(wheelVerticalGlobal)           = cross(wheelAngularVelocityGlobal, wheelVerticalGlobal);
  contactPointCoordsRelGlobal        = wheelVerticalGlobal * params.wheelHubRadius - vertical * params.wheelRadius;
  contactPointVelocityAbsGlobal      = rollerCenterVelocityGlobal + cross(rollerAngularVelocityGlobal, contactPointCoordsRelGlobal);
  contactPointVelocityAbsGlobalNorm  = norm(contactPointVelocityAbsGlobal);

  rollerVerticalGlobal               = T * vertical;
  
end RollerOnPlaneExplicit;

