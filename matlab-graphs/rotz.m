function m = rotz(rad)
    c = cos(rad);
    s = sin(rad);
    m = [
        c -s 0;
        s  c 0;
        0  0 1
    ];
