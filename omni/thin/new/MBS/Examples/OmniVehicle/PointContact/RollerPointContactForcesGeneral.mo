within MBS.Examples.OmniVehicle.PointContact;

model RollerPointContactForcesGeneral
  import ThreeD_MBS_Dynamics;
  extends RollerPointContactVelocitiesGeneral;
  parameter Real delta = 10^(-6);
  parameter Real fric = 0.1;
  Real mu;
  //  Real mu(stateSelect = StateSelect.prefer);
  Real[3] Forcet;
  //  Real[3] Forcet(start = zeros(3));
  Real Drelvn;
  Real Forcen;
  Real isInContact;
  /*
  initial equation 
  if noEvent(abs(cosBtwAxisAndVert) < cos_of_max and h < R) then
  Forcen = 1;
  else
  mu = 0;
  end if;
  */
equation
  if noEvent(abs(cosBtwAxisAndVert) < cos_of_max and InPortB.r[2] < R) then
    isInContact = 1;
    // Signorini:
  //    Drelvn = -200*relvn - 1000*h;
    Drelvn = 0;
  //    Physical:
  //    {Forcet[1], Forcet[3]} = -fric*{relvt[1], relvt[3]}*(if noEvent(relvtsqrt <= delta) then 1/delta else 1/relvtsqrt)*Forcen;
    Forcet = -fric*relvt*(if noEvent(relvtsqrt <= delta) then 1/delta else 1/relvtsqrt)*Forcen + mu*nA;
  //    Forcet[2] = 0;
  else
    isInContact = 0;
    // Signorini:
    Forcen = 0;
    Forcet = zeros(3);
  end if;

  Drelvn = der(relvn);
  Forcen = OutPortB.F*nA;

  OutPortB.F = Forcet + Forcen*nA;
  OutPortB.M = zeros(3);

  annotation (experiment(
      StopTime=10,
      NumberOfIntervals=50000,
      Tolerance=1e-009), experimentSetupOutput);
end RollerPointContactForcesGeneral;
