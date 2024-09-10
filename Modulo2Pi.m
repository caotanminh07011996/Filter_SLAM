function angle = Modulo2Pi(angleRad)
    angleTemp = mod(angleRad - pi, 2 * pi) + pi;
    angle = mod(angleTemp + pi, 2 * pi) - pi;
end