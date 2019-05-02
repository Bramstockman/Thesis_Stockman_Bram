within GIT_thesis_Bram_Stockman;
model Outside_temperature
  outer IDEAS.BoundaryConditions.SimInfoManager
                                          sim
    annotation (Placement(transformation(extent={{-98,76},{-78,96}})));
  Modelica.Blocks.Sources.RealExpression TAmb(y=sim.Te)
    annotation (Placement(transformation(extent={{-72,-4},{-52,16}})));
   Modelica.Blocks.Interfaces.RealOutput TOutside
    annotation (Placement(transformation(extent={{100,-4},{120,16}})));
equation
  connect(TAmb.y, TOutside)
    annotation (Line(points={{-51,6},{110,6}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Outside_temperature;
