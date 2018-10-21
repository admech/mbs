within MbsLite.Examples.Omni.Full;

model OmniWheelAtRest

  parameter String  name   = "OmniWheelAtRest";
  parameter Boolean strict = false;

  parameter Real[3]  Gravity (each start = inf);
  parameter Integer  nActual = -Integer_inf;
  parameter Real[3]  r0      (each start = inf);
  parameter Real[4]  q0      (each start = inf);
  parameter Params   params;
  parameter Initials initials;

  Base base;

  OmniWheel wheel
    ( name     = "wheel"
    , nActual  = nActual
    , Gravity  = Gravity
    , r0       = params.wheelRadius * vertical
    , q0       = q0
    , params   = params
    , initials = initials
    );

initial algorithm
  AssertInitialized (name, q0,      "q0");
  AssertInitialized (name, r0,      "r0");
  AssertInitialized (name, Gravity, "Gravity");

equation

  for i in 1 : nActual loop
    wheel.Rollers[i].InPorts[1].P = wheel.Rollers[i].OutPort.r;
    wheel.Rollers[i].InPorts[1].F = zeros(3);
    wheel.Rollers[i].InPorts[1].M = zeros(3);
  end for;

  wheel.InPortF.P = wheel.OutPortK.r;
  wheel.InPortF.F = zeros(3);
  wheel.InPortF.M = zeros(3);

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

end OmniWheelAtRest;

