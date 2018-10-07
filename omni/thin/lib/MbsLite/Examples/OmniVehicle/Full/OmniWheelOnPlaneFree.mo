within MbsLite.Examples.OmniVehicle.Full;

model OmniWheelOnPlaneFree

  import MbsLite.Examples.OmniVehicle.PointContact.OmniWheelOnPlane;

  parameter String  name   = "OmniWheelOnPlaneFree";
  parameter Boolean strict = false;

  parameter Real[3]  Gravity = fill(inf, 3);
  parameter Integer  nActual = -Integer_inf;
  parameter Real[3]  r0      = fill(inf, 3);
  parameter Real[4]  q0      = fill(inf, 4);
  parameter Params   params;
  parameter Initials initials;

  Base base;

  OmniWheelOnPlane wheel
    ( name     = "wheel"
    , nActual  = nActual
    , Gravity  = Gravity
    , r0       = params.wheelRadius * vertical
    , q0       = q0
    , params   = params
    , initials = initials
    );

  Real torque;
  Real epsilonNaklon;
  Real omegaNaklon;

initial algorithm
  AssertInitialized (name, q0,      "q0");
  AssertInitialized (name, r0,      "r0");
  AssertInitialized (name, Gravity, "Gravity");

equation

  wheel.InPortF.P = wheel.OutPortK.r;
  wheel.InPortF.F = zeros(3);
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
    when abs(omegaNaklon) > 1e-6 then
      assert(CompareReal(0, wheel.OutPortK.v[2]), "the wheel is nakloning! (|omegaNaklon| > 0)");
    end when;
  end if;

end OmniWheelOnPlaneFree;

