within MbsLite.Test.Omni;

model Insane
  import MbsLite.Examples.OmniVehicle.OmniVehicleParams;

  parameter OmniVehicleParams ovp;

  parameter Integer       NActual  = ovp.params.NWheels;
  // parameter Real[NActual] stuff    = { i for i in 1 : NActual };
  
  Real x;

equation

  der(x) = NActual * x;

end Insane;

