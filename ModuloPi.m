function angle = ModuloPi(angleRad)
    angleTemp = mod(angleRad - pi / 2, pi) + pi / 2;
    angle = mod(angleTemp + pi / 2, pi) - pi / 2;
end