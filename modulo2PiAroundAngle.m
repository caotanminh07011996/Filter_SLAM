function angleMod = modulo2PiAroundAngle(baseAngle, angle)
    
    angleMod = mod(angle - baseAngle + pi, 2*pi) - pi + baseAngle;
end