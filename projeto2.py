#!/usr/bin/env python

import roslib
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import numpy as np

def mensagem(T):  #recebe e trata mensagem recebida contendo transformada
	msg = geometry.msgs.msg.Transform() #inicia variavel com formatacao de transformada
	#Vamos separar a rotacao da translacao:
	rot = tf.transformations.quaternion_from_matrix(T)
	translation = tf.transformations.translation_from_matrix(T)

	#Inserindo cada vetor extraido da transformada:
	msg.translation.x = translation[0]
	msg.translation.y = translation[1]
	msg.translation.z = translation[2] #perceba que a variavel translation eh uma lista

	msg.rotation.x = rot[0]
	msg.rotation.y = rot[0]
	msg.rotation.z = rot[0]
	msg.rotation.w = rot[0] #rot tambem eh uma lista

	return msg

############################################################
# CALCULO DO ANGULO ENTRE VETORES DE N DIMENSOES


def vetor_unit(vetor): #retorna o vetor unitario do vetor
	v = vetor / np.linalg.norm(vetor) #divide o vetor pela norma do vetor (norma = modulo, tamanho do vetor)
	return v

def angulo_v(v1,v2):	#retorna o angulo (rad) entre os vetores v1 e v2
	v1_u = vetor_unit(v1)
	v2_u = vetor_unit(v2) #gerando o vetor unitario de v1 e v2
	arco = np.arccos(np.clip(np.dot(v1_u,v2_u), -1.0, 1.0))
	#np.arccos(a) retorna o arco cosseno de uma array
	#np.dot(a1,a2) faz o produto matricial de duas arrays
	#np.clip(a,i,f) limita os valores de uma array 'a' de 'i' a 'f'
	return arco

def matrizXvetor(mat,v): #multiplica matriz pelo vetor
	v.append(1)
	prod = [sum([v[x]*mat[n][x] for x in range(len(v))]) for n in range(len(mat))]
	#Detalhamento da variavel 'prod':
	#Laco 'for' selecionando item do vetor e item da matriz para que sejam multiplicados. Cada elemento de 'prod' eh formado pela soma dessas multiplicacoes, conforme ocorre em produto matricial.
	return prod

################################################################

def publicar_T():
	obj_t = geometry_msgs.msg.TransformStamped() #inicia transformada do objeto com estampa de tempo
	obj_t.header.stamp = rospy.Time.now() #estampa de tempo eh igual ao tempo atual
	obj_t.header.frame_id = "quadro_base"
	obj_t.child_frame_id = "quadro_obj"

	T1 = tf.transformation.concatenate_matrices(tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0.79,0.0,0.79)),tf.transformations.translation_matrix((0.0,1.0,1.0)))
	obj_t.transform = mensagem(T1)

	br.sendTransform(obj_t)

	#########################

	robo_t = geometry_msgs.msg.TransformStamped() #inicia transformada do robo com estampa de tempo
	robo_t.header.stamp = rospy.Time.now() #estampa de tempo eh igual ao tempo atual
	robo_t.header.frame_id = "quadro_base"
	robo_t.child_frame_id = "quadro_robo"

	T2 = tf.transformation.concatenate_matrices(tf.transformations.quaternion_matrix(tf.transformations.quaternion_about_axis(1.5,(0.0,0.0,1.0))),tf.transformations.translation_matrix((0.0,-1.0,0.0)))
	robo_t.transform = mensagem(T2)

	br.sendTransform(robo_t)

	#########################

	cam_t = geometry_msgs.msg.TransformStamped() #inicia transformada da camera com estampa de tempo
	cam_t.header.stamp = rospy.Time.now() #estampa de tempo eh igual ao tempo atual
	cam_t.header.frame_id = "quadro_robo"
	cam_t.child_frame_id = "quadro_cam"

	## Hora de calcular o vetor que aponte da camera para o objeto usando os produtos matriciais e cruzados para deduzir o angulo e o eixo de rotacao.

	global angulo
	global eixo_x
	global camera_p2
	global robo_p2
	global first_time

	#Calcular a matriz homogenea para o quadro da camera

	#matriz deslocamento do quadro da camera:
	D3 = tf.transformations.translation_matrix([0.0,0.1,0.1])
	rospy.logdebug("\n\nD3 = %s\n", D3)

	#apenas na primeira execucao, nos nao giramos o quadro da camera porque e um parametro desconhecido

	if first_time == True:
		first_time = False
		R3 = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0,0,0))

		T3 = tf.transformations.concatenate_matrices(D3,R3)
		rospy.logdebug("T3 = %s\n", T3)

		#calcular a origem das coordenadas do quadro do objeto em relacao ao quadro da camera

		p2 = tf.transformations.translation_from_matrix(T1) # A origem das coordenadas do quadro do objeto eh a parte da translacao da matriz homogenea do quadro base ao quadro do objeto

		robo_p2 = matrix_by_vector_multiplication(tf.transformations.inverse_matrix(T2),p2.tolist())
		robo_p2 = robo_p2[:(len(robo_p2)-1] #p2 com relacao ao quadro do robo


	


