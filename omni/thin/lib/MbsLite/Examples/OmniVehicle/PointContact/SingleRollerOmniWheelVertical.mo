within MbsLite.Examples.OmniVehicle.PointContact;

model SingleRollerOmniWheelVertical

  parameter Real R = 0.05 "'wheel radius' := distance from wheel axis to the floor";
  parameter Integer n = 5;
  parameter Real wheelHubMass = 0.15;
  parameter Real rollerMass = 0.05;

  parameter Real halfRollerAngle = pi / n;
  parameter Real wheelHubRadius = R * cos(halfRollerAngle);
  parameter Real rollerLength = 2 * R * sin(halfRollerAngle);
  parameter Real rollerRadiusForMoi = (R - wheelHubRadius) / 2;

  Base base;

  SingleRollerOmniWheel wheel
    ( name    = "wheel"
    , Gravity = { 0, -1, 0 }
    , n       = n
    , psi     = pi / 4
    , rollerMass            = rollerMass
    , rollerAxialMoi        = rollerMass * (rollerRadiusForMoi^2) / 2
    , rollerOrthogonalMoi   = rollerMass / 12 * (3 * rollerRadiusForMoi^2 + rollerLength^2)
    , wheelHubMass          = wheelHubMass
    , wheelHubAxialMoi      = wheelHubMass * (wheelHubRadius^2) / 2
    , wheelHubOrthogonalMoi = wheelHubMass / 12 * (3 * wheelHubRadius^2 + 0.01^2)
    , R       = R
    , r0      = { 0, R, 0 }
    , q0      = QRot(0, { 0, 0, 0 })
    , v0      = { 0, 0, 0 }
    , omega0  = { 0, 0, 0 }
    );

/*
  Verticality verticality
    ( name = "verticality"
    );
*/

equation

  connect(wheel.InPortK, base.OutPort);

/*
  connect(wheel.OutPortK, verticality.InPort);
  connect(wheel.InPortF,  verticality.OutPort);
*/
  wheel.InPortF.P = wheel.OutPortK.r;
  wheel.InPortF.F = zeros(3);
  wheel.InPortF.M = zeros(3);

end SingleRollerOmniWheelVertical;

