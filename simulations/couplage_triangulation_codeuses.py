"""
    comment coupler l'information absolue obtenue par triangulation d'une part
    et l'information relative obtenue par les roues codeuses d'autre part ?

"""

from random import *
from numpy import arange
from matplotlib.pyplot import *

#on suppose que les codeuses et l'triangulation suivent une loi normale centree
#sur la bonne valeur, avec un certain ecart type precise ci-dessous
sigma_codeuses  =   0.1
sigma_triangulation =   1


#simule une mesure de la position par triangulation
def mesure_triangulation(vraie_pos):
    return gauss(vraie_pos, sigma_triangulation)

#simule une mesure de deplacement d'une roue codeuse
def mesure_codeuse(vrai_ecart):
    return gauss(vrai_ecart, sigma_codeuses)


def generer_ligne_droite(x_max, nb_points):
    """
        genere une trajectoire rectiligne allant de 0 a x_max avec nb_points
    """

    return list(arange(0, x_max, float(x_max) / nb_points))


def simu_get_position_codeuse(traj):
    """
        pour une trajectoire reelle traj, cette fonction renvoie une trajectoire
        que pourraient donner les codeuses (donc differente de la trajectoire
        reelle car "un peu" aleatoire)
    """

    simu = traj[:]

    for i in range(1, len(traj)):
        simu[i] = simu[i - 1] + mesure_codeuse(traj[i] - traj[i - 1])


    return simu


def simu_get_position_triangulation(traj):
    """
        meme chose que simu_get_position_codeuse mais pour une position obtenue
        a partir de l'triangulation
    """

    return [mesure_triangulation(x) for x in traj]


def simu_get_position_codeuse_et_triangulation(traj):
    """
        encore la meme chose, mais en couplant l'info des codeuses et de
        l'triangulation cette fois ci
    """
    
    simu = traj[:]
    
    for i in range(1, len(traj)):
        simu[i] = ((sigma_codeuses * mesure_triangulation(traj[i]) +
                    sigma_triangulation * (mesure_codeuse(traj[i] - traj[i - 1])
                                       + simu[i - 1]))
                   / (sigma_codeuses + sigma_triangulation))
                                                                                      


    return simu


def moyenne(sig):
    return sum(sig) / float(len(sig))

def variance(sig):
    return moyenne([x * x for x in sig]) - moyenne(sig) ** 2
    

def plot_simu(x_max, N):

    vrai = generer_ligne_droite(x_max, N)
    l_x = list(range(N))
    plot(l_x, vrai, label="vrai trajectoire")

    codeuses = simu_get_position_codeuse(vrai)
    triang = simu_get_position_triangulation(vrai)
    couple = simu_get_position_codeuse_et_triangulation(vrai)

    print "codeuses : var = ", variance([codeuses[i] - vrai[i] for i in range(N)])
    print "triangulation : var = ", variance([triang[i] - vrai[i] for i in range(N)])
    print "couple : var = ", variance([couple[i] - vrai[i] for i in range(N)])
    
    plot(l_x, codeuses, label="codeuses")
    plot(l_x, triang, label="triangulation")
    plot(l_x, couple, label="couple")

    legend(loc="upper left")
    xlabel("temps")
    ylabel("position")

    show()


plot_simu(1, 50000)
    

    
