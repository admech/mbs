within MbsLite.Util.Asserts;

function AssertInitializedS

  input String modelName;
  input String value;
  input String parameterName;

algorithm
  
  assert(value <> "NOT INITIALIZED", modelName + ": " + parameterName + " is not initialized");

end AssertInitializedS;
