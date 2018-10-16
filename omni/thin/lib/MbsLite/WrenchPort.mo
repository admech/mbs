within MbsLite;

connector WrenchPort

  SI.Position[3]  P (each stateSelect = StateSelect.never);
  SI.Force[3]     F (each stateSelect = StateSelect.never);
  SI.Torque[3]    M (each stateSelect = StateSelect.never);

end WrenchPort;
