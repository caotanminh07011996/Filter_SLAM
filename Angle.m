function angle = Angle(Point1, Point2)
    X1 = Point1(1);
    Y1 = Point1(2);

    X2 = Point2(1);
    Y2 = Point2(2);

    angle = atan2(Y2 - Y1, X2 - X1);
end