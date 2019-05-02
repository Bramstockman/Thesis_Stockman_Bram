within GIT_thesis_Bram_Stockman;
model HeatingCoolingSet "Model to compute heating or cooling mode"
  parameter Real HeatSet(start=19+273);
  parameter Real CoolSet(start=20+273);
  Modelica.Blocks.Interfaces.RealOutput SetpointHeating
    "1=Heating, 0=No Heating"
    annotation (Placement(transformation(extent={{100,-2},{120,18}})));
  Modelica.Blocks.Math.BooleanToReal booleanToReal
    annotation (Placement(transformation(extent={{56,14},{76,34}})));
  Modelica.Blocks.Interfaces.RealOutput SetpointCooling
    "1=Cooling, 0=No Cooling"
    annotation (Placement(transformation(extent={{100,-34},{120,-14}})));
  Modelica.Blocks.Logical.GreaterThreshold CoolingOn(threshold = CoolSet) annotation (Placement(transformation(extent={{8,-20},{28,0}})));
  Modelica.Blocks.Math.BooleanToReal booleanToReal1
    annotation (Placement(transformation(extent={{56,-20},{76,0}})));
  Average3days1                      average3days1_1
    annotation (Placement(transformation(extent={{-84,-2},{-68,14}})));
  Modelica.Blocks.Logical.LessThreshold HeatingOn(threshold = HeatSet)
    annotation (Placement(transformation(extent={{8,14},{28,34}})));
equation

  connect(SetpointCooling, SetpointCooling)
    annotation (Line(points={{110,-24},{110,-24}}, color={0,0,127}));
  connect(average3days1_1.TRm, HeatingOn.u) annotation (Line(points={{-67.52,6},
          {-32,6},{-32,24},{6,24}}, color={0,0,127}));
  connect(CoolingOn.u, average3days1_1.TRm) annotation (Line(points={{6,-10},{-32,
          -10},{-32,6},{-67.52,6}}, color={0,0,127}));
  connect(booleanToReal.u, HeatingOn.y)
    annotation (Line(points={{54,24},{29,24}}, color={255,0,255}));
  connect(booleanToReal1.u, CoolingOn.y)
    annotation (Line(points={{54,-10},{29,-10}}, color={255,0,255}));
  connect(booleanToReal.y, SetpointHeating) annotation (Line(points={{77,24},{88,
          24},{88,8},{110,8}}, color={0,0,127}));
  connect(booleanToReal1.y, SetpointCooling) annotation (Line(points={{77,-10},{
          88,-10},{88,-24},{110,-24}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-80,66},{100,-70}},
          lineColor={28,108,200},
          lineThickness=1), Text(
          extent={{-58,36},{82,-32}},
          lineColor={28,108,200},
          textString="Heating/Cooling")}),                       Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end HeatingCoolingSet;
