within MBS.Examples.OmniVehicle.PointContact;

model RollerPointContactForces
  import ThreeD_MBS_Dynamics;
  extends RollerPointContactVelocities;
  parameter Real delta = 10^(-6);
  parameter Real fric = 0.1;
  Real mu(start = 0);
  Real[3] Forcet(start = zeros(3));
  Real Drelvn;
  Real Forcen;
  Real ForceTsqrt;
  Real w;

  Real isInContact;
equation
  w = (InPortB.T*i)*nA;

  if noEvent(abs((InPortB.T*i)*nA) < cos_of_max and h < R) then //??
  //    relvn = 0;
    Drelvn = 0;
  //    rB[2] = 0;

  //    Physical:
  //    if (noEvent(abs((InPortB.T*i)*nA) < cos(Modelica.Constants.pi/2 - alpha + 7 * Modelica.Constants.pi/180))) then
      Forcet = -fric*relvt*(if noEvent(relvtsqrt <= delta) then 1/delta else 1/relvtsqrt)*Forcen + mu*nA;
  //    else
  //      Forcet = zeros(3);
  //    end if;

  //    Shaman:
  //    Forcet = (-fric*relvt*(if noEvent(relvtsqrt <= delta) then 1/delta else 1/relvtsqrt)*Forcen + mu*nA)/(if noEvent(ForceTsqrt < 10^(-3)) then 1 else 10000000);
    isInContact = 1;
  else
    Forcen = 0;
    Forcet = zeros(3);
    isInContact = 0;
  end if;
  Drelvn = der(relvn);
  Forcen = OutPortB.F*nA;
  ForceTsqrt = sqrt(Forcet*Forcet);

  OutPortB.F = Forcet + Forcen*nA;
  OutPortB.M = zeros(3);
end RollerPointContactForces;
