within MbsLite.Examples.OmniVehicle.Full;

model OmniWheelOnPlaneFree

  import MbsLite.Examples.OmniVehicle.PointContact.OmniWheelOnPlane;

  constant Real[3] forward  = { 1,  0,  0 };
  constant Real[3] vertical = { 0,  1,  0 };
  constant Real[3] Gravity  = -vertical;
  
  parameter Boolean strict = false;

  parameter Real R = 0.05 "'wheel radius' := distance from wheel axis to the floor";
  parameter Integer n    = 5;
  parameter Integer nOne = 5;
  parameter Real wheelHubMass = 0.15;
  parameter Real rollerMass = 0.05;

  parameter Real halfRollerAngle = pi / n;
  parameter Real wheelHubRadius = R * cos(halfRollerAngle);
  parameter Real rollerLength = 2 * R * sin(halfRollerAngle);
  parameter Real rollerRadiusForMoi = (R - wheelHubRadius) / 2;
  
  parameter Real    v0          = 0;
  parameter Real    v0dirAngle  = 0;
  parameter Real[4] q0          = QRot(v0dirAngle, vertical);
  // fixme: initial orientation does not work. or does it ?
  // parameter Real[4] q0          = QRot(2*pi/5, { 0, 0, 1 });
  parameter Real[3] v0vec       = QToT(q0) * v0 * { 1, 0, 0 };

  Base base;

  OmniWheelOnPlane wheel
    ( name    = "wheel"
    , Gravity = Gravity
    , n       = n
    , nOne    = nOne
    , psi     = 0
    , rollerMass            = rollerMass
    , rollerAxialMoi        = rollerMass * (rollerRadiusForMoi^2) / 2
    , rollerOrthogonalMoi   = rollerMass / 12 * (3 * rollerRadiusForMoi^2 + rollerLength^2)
    , wheelHubMass          = wheelHubMass
    , wheelHubAxialMoi      = wheelHubMass * (wheelHubRadius^2) / 2
    , wheelHubOrthogonalMoi = wheelHubMass / 12 * (3 * wheelHubRadius^2 + 0.01^2)
    , R       = R
    , r0      = R * vertical
    , q0      = q0
    , v0      = v0vec
    , omega0  = 1 / R * cross(vertical, v0vec)
    );

  Real torque;
  Real epsilonNaklon;
  Real omegaNaklon;

equation

  wheel.InPortF.P = wheel.OutPortK.r;
  wheel.InPortF.F = zeros(3);
  /*
  wheel.InPortF.M = zeros(3);
  */
  wheel.InPortF.M = torque * wheel.OutPortK.T * forward;
  omegaNaklon = wheel.OutPortK.omega * (wheel.OutPortK.T * forward);
  der(omegaNaklon) = epsilonNaklon;
  epsilonNaklon = 0;

  connect(wheel.InPortK, base.OutPort);

  if strict then
    when wheel.OutPortK.r[2] < R then
      assert(CompareReal(R, wheel.OutPortK.r[2]), "the wheel is falling down! (r[2] < R)");
    end when;
    when wheel.OutPortK.r[2] > R then
      assert(CompareReal(R, wheel.OutPortK.r[2]), "the wheel is flying! (r[2] > R)");
    end when;
    when wheel.OutPortK.v[2] < 0 then
      assert(CompareReal(0, wheel.OutPortK.v[2]), "the wheel is falling down! (v[2] < 0)");
    end when;
    when wheel.OutPortK.v[2] > 0 then
      assert(CompareReal(0, wheel.OutPortK.v[2]), "the wheel is flying! (v[2] > 0)");
    end when;
  end if;

end OmniWheelOnPlaneFree;

