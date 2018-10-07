within MbsLite.Examples.OmniVehicle.Contact;

model PlaneContact

  parameter Params   params;
  parameter Integer  nActual;

  KinematicPort            wheelOutPort;
  WrenchPort               wheelInPort;
  KinematicPort[nActual]   rollerOutPorts;
  WrenchPort[nActual]      rollerInPorts;

protected
  Integer   lowestRollerIndex;
  Real[3]   coords;
  Real[3]   velocity;

/*
  Real      normalVelocity;
  Real      DnormalVelocity;
*/

equation

  lowestRollerIndex = Argmin
    ( { rollerOutPorts[i].r[2] for i in 1 : nActual }
    );

  (coords, velocity) = FindLowestPoint
    ( wheelOutPort.r
    , params.wheelRadius
    , rollerOutPorts[lowestRollerIndex].r
    , rollerOutPorts[lowestRollerIndex].v
    , rollerOutPorts[lowestRollerIndex].omega
    );

/*
  normalVelocity = velocity * vertical;
  der(normalVelocity) = DnormalVelocity;
  DnormalVelocity = 0;
*/

  for i in 1 : nActual loop
    rollerInPorts[i].P = rollerOutPorts[i].r;
    rollerInPorts[i].M = zeros(3);
    if i == lowestRollerIndex then
      rollerInPorts[i].F[1] = 0;
      rollerInPorts[i].F[3] = 0;
      rollerInPorts[i].F[2] = 0; // debugging !
    else
      rollerInPorts[i].F = zeros(3);
    end if;
  end for;

  wheelInPort.P = wheelOutPort.r;
  wheelInPort.F = zeros(3);
  wheelInPort.M = zeros(3);

end PlaneContact;

