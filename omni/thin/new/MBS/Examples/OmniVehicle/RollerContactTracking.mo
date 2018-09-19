within MBS.Examples.OmniVehicle;

partial model RollerContactTracking
  extends Constraint;
  parameter Integer n = 4 "Number of rollers";
  parameter Real alpha = Modelica.Constants.pi/n
    "Max angle of the half-sector";
  parameter Real cos_of_max = cos(Modelica.Constants.pi/2 - alpha); // cos(pi/2 - pi/n)
  parameter Real R = 1 "Omni wheel outer radius";
  parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
  parameter Real L1 = R*sin(alpha) "Half roller length";
  parameter Real[3] nA = {0, 1, 0}
    "Roller over horizontal surface: vertical unit vector";
  parameter Real[3] i = {1, 0, 0} "Roller axis of symmetry unit vector";
  parameter Real psi = 0 "Angle of roller distortion (fixed axis turn)";
  SI.Position[3] rA;
  SI.Position[3] rB;
//  Real[3] b;
  Real[3] c;
  Real[3] d;
  Real h;
  Real j;
  Real cosBtwAxisAndVert;
equation
//  b = cross(InPortB.T*i, i);
  d = cross(InPortB.T*i, nA);
  cosBtwAxisAndVert = (InPortB.T*i)*nA;

  if noEvent(abs(cosBtwAxisAndVert) < cos_of_max) then
     c = R1*cross(d, InPortB.T*i)/sqrt(d*d);
     rB = InPortB.r + c - R*nA;
     j = 1; // IN CONTACT
  else
     if noEvent(cosBtwAxisAndVert > 0) then
       c = -L1*InPortB.T*i;
       j = 2; // ??? roller is to the right of vertical
     else
       c = L1*InPortB.T*i;
       j = 3; // ??? roller is to the left of vertical
     end if;
     rB = InPortB.r + c;
  end if;

  rA = {rB[1], 0, rB[3]};
  h = rB[2];
end RollerContactTracking;
