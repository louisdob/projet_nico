#!/usr/bin/env python

import rospy
import cv2
import numpy as np
#from std_msgs.msg import Float64
#from std_msgs.msg import Int32

from projet_nico.msg import Position
from std_msgs.msg import String

# -------------- variabe ----------------
image = None


def traitement_objet():

    # ouvre la camera par default il me sembel -1 ou 0

    # ---------------POur Tester-------------
    #pub = rospy.Publisher('position_topic', Position, queue_size=10)
    #rate = rospy.Rate(1)
    #rospy.loginfo('Publisher :')
    #msg = Position()
    #msg.x = 1
    #msg.y = 2
    #msg.color = 1
    # while not rospy.is_shutdown():
    # pub.publish(msg)
    # rate.sleep()
    # ------fin publication

    # -------------fin Test-----------------

    #img = cv2.imread('test2.jpg')
    # cv2.imshow("clasique",img)

    video_stream = cv2.VideoCapture(0)
    #ret, frame = videostream.retrieve()
    _, frame = video_stream.read()

    img = frame

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    imgGrey = cv2.cvtColor(hsv_img, cv2.COLOR_BGR2GRAY)  # noir et blanc

    # cv2.imshow("hsv",hsv_img)
    # modele hsv afin de determiner la teinte la saturation et la valeur bien mieux pour trouver les couleurs
    _, thrash = cv2.threshold(imgGrey, 80, 255, cv2.THRESH_BINARY)
    # cv2.imshow("noir_blanc",imgGrey)
    # cv2.imshow("noir_blanc2",thrash)
    # _, threshold =cv2.threshold(img,245,250,cv2.THRESH_BINARY)#met tous les pixels entre 245 et 250 uniquement : que du noir
    # threshold valeur de seuil

    lower_blue = np.array([90, 50, 85])
    upper_blue = np.array([126, 255, 255])
    mask = cv2.inRange(hsv_img, lower_blue, upper_blue)
    cv2.imshow("rouge", mask)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    # le NONE prends tous les points qui coutrne une forme : les coutours
    for cnt in contours:

        c = cv2.contourArea(cnt)  # prends que les grandes formes
        # print(c)
        if ((c >= 1000) and (c <= 10000)):
            print(c)
            approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
            # nous travons maintenant tous les contours en noir (le 0)
            cv2.drawContours(img, [approx], 0, (0), 5)

            # -----------centre------------
            M = cv2.moments(cnt)  # moment d'ordre 3 du polygone
            cx = int(M['m10']/M['m00'])  # m00 aire de la forme
            cy = int(M['m01']/M['m00'])  # cx et cy centre d'une forme

            # ---------partie couleur----------------------

            # --masque pour voir uniquement certaines couleurs

            # lower_red=np.array([161,155,81])
            # upper_red=np.array([179,255,255])

            # lower_green=np.array([25,52,72])
            # upper_green=np.array([102,255,255])

            # cv2.imshow("rouge",mask)

            # nous recuperons le centre de chaque contour dans l'image hsv
            forme = hsv_img[cy, cx]

            # recuperation de la teinte entre 0 et 360 : partie du modele de couleur
            teinte = forme[0]
            # print(forme)

            # mets un point au centre
            cv2.circle(img, (cx, cy), 1, (255, 0, 0), 3)
            print(cx, cy)  # affiche les valeurs du centre

            x = approx.ravel()[0]  # coordonnees au dessus de chaque forme
            y = approx.ravel()[1]-10

            if teinte < 5:
                cv2.putText(img, "Rouge", (x, y-20),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, (0))
                msg.color = 1  # 1 pour rouge
            elif 40 < teinte < 78:
                cv2.putText(img, "Vert", (x, y-20),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, (0))
                msg.color = 2  # 2 pour vert
            elif 90 < teinte < 130:
                cv2.putText(img, "Bleu", (x, y-20),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, (0))
                msg.color = 3  # 3 pour bleu
            else:
                cv2.putText(img, "Rouge", (x, y-20),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, (0))
                msg.color = 1

            # -----------------forme-------------
            if len(approx) == 3:
                cv2.putText(img, "Triangle", (x, y),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, (0))

            elif len(approx) == 4:

                cv2.putText(img, "quadrilatere", (x, y),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, (0))
            # cordonnees police de cractere, taille et couleur
            elif len(approx) == 5:
                cv2.putText(img, "pentagone", (x, y),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, (0))
            else:
                cv2.putText(img, "Cercle", (x, y),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, (0))

            # publication du message--------------
            pub = rospy.Publisher('position_topic', Position, queue_size=10)
            rate = rospy.Rate(1)
            rospy.loginfo('Publisher :')
            msg = Position()
            msg.x = cx
            msg.y = cy

            while not rospy.is_shutdown():
                pub.publish(msg)
                rate.sleep()
            # ------fin publication

            cv2.imshow("shapes", img)
            cv2.waitKey(0)


def my_script():
    rospy.init_node('detection', anonymous=True)
    # print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
    traitement_objet()
    # print("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb")
    rospy.spin()


if __name__ == '__main__':
    try:
        my_script()
    except rospy.ROSInterruptException:
        pass
