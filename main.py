import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sklearn
from imutils import contours
import cv2
from PIL import Image,ImageGrab
import kociemba
import keyboard

vid = cv2.VideoCapture(0)
p=[(0,0,0)]*96
p1=[(0,0,0)]*96
p0=['O']*96
pf=['O']*54
z=90
z1=180
n=0
m='ru'
def sol(pj):
    for i in range(0,42):
        pj.remove('BL')
    pf = pj[0:54]
    print(pf)
    km =''.join(map(str,pf))
    print(km)
    soln = kociemba.solve(km)
    print(soln)
def mode(p):
    p[3]=(300,300,300)
    p[7]=(300,300,300)
    p[11]=(300,300,300)
    p[12]=(300,300,300)
    p[13]=(300,300,300)
    p[14]=(300,300,300)
    p[15]=(300,300,300)
    p[16+3] = (300, 300, 300)
    p[16+7] = (300, 300, 300)
    p[16+11] = (300, 300, 300)
    p[16+12] = (300, 300, 300)
    p[16+13] = (300, 300, 300)
    p[16+14] = (300, 300, 300)
    p[16+15] = (300, 300, 300)
    p[32+3] = (300, 300, 300)
    p[32+7] = (300, 300, 300)
    p[32+11] = (300, 300, 300)
    p[32+12] = (300, 300, 300)
    p[32+13] = (300, 300, 300)
    p[32+14] = (300, 300, 300)
    p[32+15] = (300, 300, 300)
    p[48+3] = (300, 300, 300)
    p[48+7] = (300, 300, 300)
    p[48+11] = (300, 300, 300)
    p[48+12] = (300, 300, 300)
    p[48+13] = (300, 300, 300)
    p[48+14] = (300, 300, 300)
    p[48+15] = (300, 300, 300)
    p[64+3] = (300, 300, 300)
    p[64+7] = (300, 300, 300)
    p[64+11] = (300, 300, 300)
    p[64+12] = (300, 300, 300)
    p[64+13] = (300, 300, 300)
    p[64+14] = (300, 300, 300)
    p[64+15] = (300, 300, 300)
    p[80+3] = (300, 300, 300)
    p[80+7] = (300, 300, 300)
    p[80+11] = (300, 300, 300)
    p[80+12] = (300, 300, 300)
    p[80+13] = (300, 300, 300)
    p[80+14] = (300, 300, 300)
    p[80+15] = (300, 300, 300)
    return p
def Reverse(tuples):
    new_tup = tuples[::-1]
    return new_tup
def rgb_to_hsv(r, g, b):
    k ='N'
    r, g, b = r/255.0, g/255.0, b/255.0
    mx = max(r, g, b)
    mn = min(r, g, b)
    df = mx-mn
    if mx == mn:
        h = 0
    elif mx == r:
        h = (60 * ((g-b)/df) + 360) % 360
    elif mx == g:
        h = (60 * ((b-r)/df) + 120) % 360
    elif mx == b:
        h = (60 * ((r-g)/df) + 240) % 360
    if mx == 0:
        s = 0
    else:
        s = (df/mx)*100
    v = mx*100
    if v<30:
        k='D'
    elif (h >300 or h<1) and s>10:
        k='L'
    elif (h<40 and h>=1) and s>10:
        k='R'
    elif h<70 and h>30 and s>10:
        k='U'
    elif h<275 and h>200 and s>10:
        k='B'
    elif h>100 and h<190 and s>10:
        k='F'
    else:
        k='B'
    if v>100:
        k = 'BL'
    return  k



while (True):


    ret, frame = vid.read()

    image = cv2.rectangle(frame, (200,200), (220,220), (155, 122, 20), 1)
    image = cv2.rectangle(image, (240, 240), (260, 260), (155, 122, 20), 1)
    image = cv2.rectangle(image, (280, 280), (300, 300), (155, 122, 20), 1)
    image = cv2.rectangle(image, (320, 320), (340, 340), (155, 122, 20), 1)
    image = cv2.rectangle(image, (200, 240), (220, 260), (155, 122, 20), 1)
    image = cv2.rectangle(image, (200, 280), (220, 300), (155, 122, 20), 1)
    image = cv2.rectangle(image, (200, 320), (220, 340), (155, 122, 20), 1)
    image = cv2.rectangle(image, (240, 200), (260, 220), (155, 122, 20), 1)
    image = cv2.rectangle(image, (280, 200), (300, 220), (155, 122, 20), 1)
    image = cv2.rectangle(image, (320, 200), (340, 220), (155, 122, 20), 1)
    image = cv2.rectangle(image, (320, 280), (340, 300), (155, 122, 20), 1)
    image = cv2.rectangle(image, (280, 240), (300, 260), (155, 122, 20), 1)
    image = cv2.rectangle(image, (320, 240), (340, 260), (155, 122, 20), 1)
    image = cv2.rectangle(image, (240, 320), (260, 340), (155, 122, 20), 1)
    image = cv2.rectangle(image, (240, 280), (260, 300), (155, 122, 20), 1)
    image = cv2.rectangle(image, (280, 320), (300, 340), (155, 122, 20), 1)
    image = cv2.rectangle(image, (5, 5), (15, 15), p1[0], 7)
    image = cv2.rectangle(image, (25,25), (35, 35), p1[5] , 7)
    image = cv2.rectangle(image, (45, 45), (55, 55), p1[10], 7)
    image = cv2.rectangle(image, (65, 65), (75, 75), p1[15], 7)
    image = cv2.rectangle(image, (5, 25), (15, 35), p1[4], 7)
    image = cv2.rectangle(image, (5, 45), (15, 55), p1[8], 7)
    image = cv2.rectangle(image, (5, 65), (15, 75), p1[12], 7)
    image = cv2.rectangle(image, (25, 5), (35, 15), p1[1], 7)
    image = cv2.rectangle(image, (45, 5), (55, 15), p1[2], 7)
    image = cv2.rectangle(image, (65, 5), (75, 15), p1[3], 7)
    image = cv2.rectangle(image, (65, 45), (75, 55), p1[11], 7)
    image = cv2.rectangle(image, (45, 25), (55, 35), p1[6], 7)
    image = cv2.rectangle(image, (65, 25), (75, 35), p1[7], 7)
    image = cv2.rectangle(image, (25, 65), (35, 75), p1[13], 7)
    image = cv2.rectangle(image, (25, 45), (35, 55), p1[9], 7)
    image = cv2.rectangle(image, (45, 65), (55, 75), p1[14], 7)
    image = cv2.rectangle(image, (5+z, 5), (15+z, 15), p1[16], 7)
    image = cv2.rectangle(image, (25+z, 25), (35+z, 35), p1[21], 7)
    image = cv2.rectangle(image, (45+z, 45), (55+z, 55), p1[26], 7)
    image = cv2.rectangle(image, (65+z, 65), (75+z, 75), p1[31], 7)
    image = cv2.rectangle(image, (5+z, 25), (15+z, 35), p1[20], 7)
    image = cv2.rectangle(image, (5+z, 45), (15+z, 55), p1[24], 7)
    image = cv2.rectangle(image, (5+z, 65), (15+z, 75), p1[28], 7)
    image = cv2.rectangle(image, (25+z, 5), (35+z, 15), p1[17], 7)
    image = cv2.rectangle(image, (45+z, 5), (55+z, 15), p1[18], 7)
    image = cv2.rectangle(image, (65+z, 5), (75+z, 15), p1[19], 7)
    image = cv2.rectangle(image, (65+z, 45), (75+z, 55), p1[27], 7)
    image = cv2.rectangle(image, (45+z, 25), (55+z, 35), p1[22], 7)
    image = cv2.rectangle(image, (65+z, 25), (75+z, 35), p1[23], 7)
    image = cv2.rectangle(image, (25+z, 65), (35+z, 75), p1[29], 7)
    image = cv2.rectangle(image, (25+z, 45), (35+z, 55), p1[25], 7)
    image = cv2.rectangle(image, (45+z, 65), (55+z, 75), p1[30], 7)
    image = cv2.rectangle(image, (5 + z1, 5), (15 + z1, 15), p1[32], 7)
    image = cv2.rectangle(image, (25 + z1, 25), (35 + z1, 35), p1[37], 7)
    image = cv2.rectangle(image, (45 + z1, 45), (55 + z1, 55), p1[42], 7)
    image = cv2.rectangle(image, (65 + z1, 65), (75 + z1, 75), p1[47], 7)
    image = cv2.rectangle(image, (5 + z1, 25), (15 + z1, 35), p1[36], 7)
    image = cv2.rectangle(image, (5 + z1, 45), (15 + z1, 55), p1[40], 7)
    image = cv2.rectangle(image, (5 + z1, 65), (15 + z1, 75), p1[44], 7)
    image = cv2.rectangle(image, (25 + z1, 5), (35 + z1, 15), p1[33], 7)
    image = cv2.rectangle(image, (45 + z1, 5), (55 + z1, 15), p1[34], 7)
    image = cv2.rectangle(image, (65 + z1, 5), (75 + z1, 15), p1[35], 7)
    image = cv2.rectangle(image, (65 + z1, 45), (75 + z1, 55), p1[43], 7)
    image = cv2.rectangle(image, (45 + z1, 25), (55 + z1, 35), p1[38], 7)
    image = cv2.rectangle(image, (65 + z1, 25), (75 + z1, 35), p1[39], 7)
    image = cv2.rectangle(image, (25 + z1, 65), (35 + z1, 75), p1[45], 7)
    image = cv2.rectangle(image, (25 + z1, 45), (35 + z1, 55), p1[41], 7)
    image = cv2.rectangle(image, (45 + z1, 65), (55 + z1, 75), p1[46], 7)
    image = cv2.rectangle(image, (5, 5+z), (15, 15+z), p1[48], 7)
    image = cv2.rectangle(image, (25, 25+z), (35, 35+z), p1[53], 7)
    image = cv2.rectangle(image, (45, 45+z), (55, 55+z), p1[58], 7)
    image = cv2.rectangle(image, (65, 65+z), (75, 75+z), p1[63], 7)
    image = cv2.rectangle(image, (5, 25+z), (15, 35+z), p1[52], 7)
    image = cv2.rectangle(image, (5, 45+z), (15, 55+z), p1[56], 7)
    image = cv2.rectangle(image, (5, 65+z), (15, 75+z), p1[60], 7)
    image = cv2.rectangle(image, (25, 5+z), (35, 15+z), p1[49], 7)
    image = cv2.rectangle(image, (45, 5+z), (55, 15+z), p1[50], 7)
    image = cv2.rectangle(image, (65, 5+z), (75, 15+z), p1[51], 7)
    image = cv2.rectangle(image, (65, 45+z), (75, 55+z), p1[59], 7)
    image = cv2.rectangle(image, (45, 25+z), (55, 35+z), p1[54], 7)
    image = cv2.rectangle(image, (65, 25+z), (75, 35+z), p1[55], 7)
    image = cv2.rectangle(image, (25, 65+z), (35, 75+z), p1[61], 7)
    image = cv2.rectangle(image, (25, 45+z), (35, 55+z), p1[57], 7)
    image = cv2.rectangle(image, (45, 65+z), (55, 75+z), p1[62], 7)
    image = cv2.rectangle(image, (5 + z1, 5+z), (15 + z1, 15+z), p1[80], 7)
    image = cv2.rectangle(image, (25 + z1, 25+z), (35 + z1, 35+z), p1[85], 7)
    image = cv2.rectangle(image, (45 + z1, 45+z), (55 + z1, 55+z), p1[90], 7)
    image = cv2.rectangle(image, (65 + z1, 65+z), (75 + z1, 75+z), p1[95], 7)
    image = cv2.rectangle(image, (5 + z1, 25+z), (15 + z1, 35+z), p1[84], 7)
    image = cv2.rectangle(image, (5 + z1, 45+z), (15 + z1, 55+z), p1[88], 7)
    image = cv2.rectangle(image, (5 + z1, 65+z), (15 + z1, 75+z), p1[92], 7)
    image = cv2.rectangle(image, (25 + z1, 5+z), (35 + z1, 15+z), p1[81], 7)
    image = cv2.rectangle(image, (45 + z1, 5+z), (55 + z1, 15+z), p1[82], 7)
    image = cv2.rectangle(image, (65 + z1, 5+z), (75 + z1, 15+z), p1[83], 7)
    image = cv2.rectangle(image, (65 + z1, 45+z), (75 + z1, 55+z), p1[91], 7)
    image = cv2.rectangle(image, (45 + z1, 25+z), (55 + z1, 35+z), p1[86], 7)
    image = cv2.rectangle(image, (65 + z1, 25+z), (75 + z1, 35+z), p1[87], 7)
    image = cv2.rectangle(image, (25 + z1, 65+z), (35 + z1, 75+z), p1[93], 7)
    image = cv2.rectangle(image, (25 + z1, 45+z), (35 + z1, 55+z), p1[89], 7)
    image = cv2.rectangle(image, (45 + z1, 65+z), (55 + z1, 75+z), p1[94], 7)
    image = cv2.rectangle(image, (5 + z, 5 + z), (15 + z, 15 + z), p1[64], 7)
    image = cv2.rectangle(image, (25 + z, 25 + z), (35 + z, 35 + z), p1[69], 7)
    image = cv2.rectangle(image, (45 + z, 45 + z), (55 + z, 55 + z), p1[74], 7)
    image = cv2.rectangle(image, (65 + z, 65 + z), (75 + z, 75 + z), p1[79], 7)
    image = cv2.rectangle(image, (5 + z, 25 + z), (15 + z, 35 + z), p1[68], 7)
    image = cv2.rectangle(image, (5 + z, 45 + z), (15 + z, 55 + z), p1[72], 7)
    image = cv2.rectangle(image, (5 + z, 65 + z), (15 + z, 75 + z), p1[76], 7)
    image = cv2.rectangle(image, (25 + z, 5 + z), (35 + z, 15 + z), p1[65], 7)
    image = cv2.rectangle(image, (45 + z, 5 + z), (55 + z, 15 + z), p1[66], 7)
    image = cv2.rectangle(image, (65 + z, 5 + z), (75 + z, 15 + z), p1[67], 7)
    image = cv2.rectangle(image, (65 + z, 45 + z), (75 + z, 55 + z), p1[75], 7)
    image = cv2.rectangle(image, (45 + z, 25 + z), (55 + z, 35 + z), p1[70], 7)
    image = cv2.rectangle(image, (65 + z, 25 + z), (75 + z, 35 + z), p1[71], 7)
    image = cv2.rectangle(image, (25 + z, 65 + z), (35 + z, 75 + z), p1[77], 7)
    image = cv2.rectangle(image, (25 + z, 45 + z), (35 + z, 55 + z), p1[73], 7)
    image = cv2.rectangle(image, (45 + z, 65 + z), (55 + z, 75 + z), p1[78], 7)
    image = cv2.line(image,(195,197),(195,307),color=(150,147,148),thickness=2)
    image = cv2.line(image, (195, 307), (305, 307), color=(150, 147, 148), thickness=2)
    image = cv2.line(image, (305, 307), (305, 197), color=(150, 147, 148), thickness=2)
    image = cv2.line(image, (195, 197), (305, 197), color=(150, 147, 148), thickness=2)
    image = cv2.putText(image,'3x3',(280,190),cv2.FONT_HERSHEY_SIMPLEX,0.6,(150,140,140),1)
    image = cv2.putText(image, 'For 3x3 mode press m then start', (300, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (250, 140, 140), 2)
    image = cv2.putText(image, 'U R F D L B', (300, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 110, 110), 1)
    cv2.imshow('frame',image)
    k=cv2.waitKey(33)
    if k == ord('m'):
        m='ra'
    if k==ord('c'):
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        im_pil = Image.fromarray(img)

        p[16*n+0]= im_pil.getpixel((210, 210))
        p[16*n+5] = im_pil.getpixel((250, 250))
        p[16*n+10] = im_pil.getpixel((290, 290))
        p[16*n+15] = im_pil.getpixel((330, 330))
        p[16*n+4] = im_pil.getpixel((210, 250))
        p[16*n+8] = im_pil.getpixel((210, 290))
        p[16*n+12] = im_pil.getpixel((210, 330))
        p[16*n+1] = im_pil.getpixel((250, 210))
        p[16*n+2] = im_pil.getpixel((290, 210))
        p[16*n+3] = im_pil.getpixel((330, 210))
        p[16*n+11] = im_pil.getpixel((330, 290))
        p[16*n+6] = im_pil.getpixel((290, 250))
        p[16*n+7] = im_pil.getpixel((330, 250))
        p[16*n+13] = im_pil.getpixel((250, 330))
        p[16*n+9] = im_pil.getpixel((250, 290))
        p[16*n+14] = im_pil.getpixel((290, 330))
        if m=='ra':
            p=mode(p)
        p1 = list(map(lambda x: Reverse(x), p))
        p0 = list(map(lambda x: rgb_to_hsv(x[0],x[1],x[2]), p))
        print(p)
        print(p1)
        print(p0)
    if k == ord('d'):
        n=n+1;
        print(n)
    if k == ord('q'):
        break
    if k==ord('s'):
        print(sol(p0))
        break








cv2.waitKey()
vid.release()
cv2.destroyAllWindows()
