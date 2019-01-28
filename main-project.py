#!/usr/bin/python
# coding: utf-8
 
 
#----------------------------------------------------------------
# Autor: Saymon C. A. Oliveira
# Email: saymowan@gmail.com
# Descrição: este algoritmo descreve a 13ª implementação de OpenCV
# Funções: Imagem digital -> Transformação HSV -> Imagem binária -> Erosão binária -> Encontrar área -> Encontrar coordenadas ->
# Funções 2: desenhar circulo no centroide (x,y) -> declaração de pinos -> declaração de funções de movimento do motor -> 
# Funções 3: declaração Função Z -> declaração Função X -> configurar servo motor -> declaração Função Y -> modularização do SVC*
# Funções 4: realizar movimento do robô  de profundidade (area) e vertical (y) -> 3 ajustes -> FEEDBACK DE ESTADOS -> Multiplos Contornos
# Tecnologias: OpenCV, Python, GPIO e NumPy
# Mais informações: Seção 3.4 Testes
#---------------------------------------------------------------
 
#SVC - Sistema de Visão Computacional
 
 
''' Informações do script
Autor:  Saymon C. A. Oliveira
Email: saymowan@gmail.com
 
Este script descreve as funções de locomoção e ajuste junto com
um recurso de pesquisar multiplos objetos e definir um para ser seguido...
Se o objeto for abaixo de 50px ou não é o maior....
é ignorado e seu formato não atrapalha no reconhecimento.
 
Extras: implementar cores para o contorno que não são "o ator principal"
 
 
 
'''
 
 
import cv2.cv as cv
import cv2 as cv2
import time
import numpy as np
import RPi.GPIO as gpio
import math
 
# Tipo de GPIO
gpio.setmode(gpio.BOARD)
# Desliga alertas
gpio.setwarnings(False)
 
#---------------------------------------------------------------------
#   R E L A T O R I O S
#---------------------------------------------------------------------
 
#---------------------------------------------------------------------
#  C O N F I G U R A C A O   D O    S E R V O 
#---------------------------------------------------------------------
def set(property, value):
    try:
        f = open("/sys/class/rpi-pwm/pwm0/" + property, 'w')
        f.write(value)
        f.close()   
    except:
        print("Error writing to: " + property + " value: " + value)
         
def setServo(angle):
    set("servo", str(angle))
     
set("delayed", "0")
set("mode", "servo")
set("servo_max", "160")  #160
set("active", "1")
servo = 100
servoMin = 30
servoMax = 160
setServo(servo)
 
 
 
#---------------------------------------------------------------------------
#   F U N Ç Ã O   D E   A J U S T E   V E R T I C A L
#---------------------------------------------------------------------------
# Parametros:
# y -> posição atual no eixo Y
# may -> limite do centro
# centroy -> posição estática do centro
 
def ajusteY(y, may, centroy, servo):
       # Se o limite for maior que a distancia do centro que é considerado centralizado
       if (y - centroy) >= may:
        servo = servo - 5
    setServo(servo)
         
       # Se o limite for menor ...
       elif (centroy - y) >= may:
        servo = servo + 5
    setServo(servo)
         
       #Se o  valor de deslocamento for inferior ao Minimo, faz o minimo
       if servo < servoMin:
        servo = servoMin
    setServo(servo)
         
       # Se o valor de deslocamento for superior ao Maximo, faz o maximo
       if servo > servoMax:
        servo = servoMax
        setServo(servo)
         
 
       
#--------------------------------------------------------------------------
#  A T I V A Ç Ã O   D O S     M O T O R E S 
#--------------------------------------------------------------------------
#Declara pinos como saida GPIO - Motor A
  
#pino de ativação do motor A via Rasp 1
gpio.setup(7, gpio.OUT)
  
#pino de ativação do motor A via Rasp 2 
gpio.setup(11, gpio.OUT)
  
# Iniciar Pino 13 como saida - Motor A
gpio.setup(13, gpio.OUT)
  
#Iniciar Pino 15 como saida - Motor A
gpio.setup(15, gpio.OUT)
  
#---------------------------------------
#Declara pinos como saida GPIO - Motor B
  
#pino de ativação do motor B via Rasp 1
gpio.setup(26, gpio.OUT)
  
#pino de ativação do motor B via Rasp 2 
gpio.setup(16, gpio.OUT)
  
# Iniciar Pino 5 como saida - Motor B
gpio.setup(18, gpio.OUT)
  
#Iniciar Pino 22 como saida - Motor B
gpio.setup(22, gpio.OUT)
  
 
 
#-----------------------------------------
# Permitir que o L298N seja controlado pelo GPIO:
#---------------------------------------
#Valores iniciais - True - Motor A ativado
gpio.output(7, True) #Motor A - Rasp 1
gpio.output(11, True) #Motor A - Rasp 2
#---------------------------------------
#Valores iniciais - True - Motor B ativado
gpio.output(26, True) #Motor B - Rasp 1
gpio.output(16, True) #Motor B - Rasp 2
#---------------------------------------
 
def Frente():
# Motor 1
 gpio.output(13, True)
 gpio.output(15, False)
# Motor 2
 gpio.output(18, False)
 gpio.output(22, True)
     
def Tras():
# Motor 1
 gpio.output(13, False)
 gpio.output(15, True)
# Motor 2
 gpio.output(18, True)
 gpio.output(22, False)
  
def Parar():
# Motor 1
 gpio.output(18, False)
 gpio.output(22, False)
# Motor 2
 gpio.output(13, False)
 gpio.output(15, False)
 
def Direita():
# Motor 1
 gpio.output(13, True)
 gpio.output(15, False)
# Motor 2
 gpio.output(18, True)
 gpio.output(22, False)
 
def Esquerda():
# Motor 1
 gpio.output(13, False)
 gpio.output(15, True)
# Motor 2
 gpio.output(18, False)
 gpio.output(22, True)
 
  
#---------------------------------------------------------------------------
#   F U N Ç Ã O   D E   A J U S T E   H O R I  Z O N T A L
#---------------------------------------------------------------------------
# Parametros:
# x -> posição atual no eixo X
# max -> limite do centro
# centrox -> posição estática do centro
 
def ajusteX(x, y, max, centrox):
    # Se a  distância na parte direita for maior que o limite é feito o ajsute
       if (x - centrox) > max:
        cv2.line(entrada, (int(x),int(y)), (centrox,centroy),(0,0,255), 1)
        #movimento = 'Horizontal Direita'
            Direita()
        time.sleep(0.05)
        Parar()
        
       # Se a distância na parte esquerda for maior que o limite é feito o ajuste
       elif (centrox - x) > max:
        cv2.line(entrada, (int(x),int(y)), (centrox,centroy),(0,0,255), 1)
        #movimento = 'Horizontal Esquerda'
        Esquerda()
    time.sleep(0.05)
    Parar()
         
       else: 
     Parar()
#--------------------------------------------------------------------------
  
#----------------------------------------------------------------
#    P R O C E S S A M E N T O   D E   I M A G E N S
#------------------------------------------------------------------
# USAR FUNÇÃO INRANGE PARA MUDAR DE RGB-HSV
# PARA ISSO TEMOS QUE DEFINIR OS LIMITES DE VALORES DE H,S E VALORES
 
# Faixa de HSV que usamos para detectar o objeto colorido
# Neste exemplo, pré definidos para uma bola verde
Hmin = 42
Hmax = 92
Smin = 62
Smax = 255
Vmin = 63
Vmax = 235
 
# Cria-se um array de valores HSV(mínimo e máximo)
rangeMin = np.array([Hmin, Smin, Vmin], np.uint8)
rangeMax = np.array([Hmax, Smax, Vmax], np.uint8)
 
# Funções de processamento  da imagem
def processamento(entrada):
     imgMedian = cv2.medianBlur(entrada,1)
     imgHSV = cv2.cvtColor(imgMedian,cv2.cv.CV_BGR2HSV) 
     #imgErode = cv2.inRange(imgHSV, rangeMin, rangeMax)
     imgThresh = cv2.inRange(imgHSV, rangeMin, rangeMax)
     imgErode = cv2.erode(imgThresh, None, iterations = 3)
     return imgErode
      
#-------------------------------------------------------------------------
cv.NamedWindow("Entrada")
#cv.NamedWindow("HSV")
#cv.NamedWindow("Thre")
cv.NamedWindow("Erosao")
capture = cv2.VideoCapture(0)
#------------------------------------------------------------------------
#  P A D R O E S     D E      A J U S T E
#------------------------------------------------------------------------  
# Parametros do tamannho da imagem de captura
largura = 160
altura = 120
 
 
 
# Area minima a ser detectada
minArea = 50 #cerca de 80 cm
 
#Centro dos eixos
centroy = altura/2
centrox = largura/2
 
# Limite do centro
may = altura/5 #24
max = largura/2.5  # 64 pixels
#----------------------------------------------------------------------
 
 
 
# Definir um tamanho para os frames (descartando o PyramidDown)
if capture.isOpened():
  capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, largura)
  capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, altura)
 
 
melhor_contorno=[]
# Loop de reconhecimento e tomada de decisões
while True:
    ret, entrada = capture.read()
    imagem_processada = processamento(entrada)
    cv2.imshow("Erosao", imagem_processada)
    # verificar parâmetros de contornos (N > 1)
    contours,hierarchy = cv2.findContours(imagem_processada,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
 
    #Lista de contornos e definir o maior (tratando N objetos no frame) 
    maior_area = 0
    for posicao_contorno in contours:
        area = cv2.contourArea(posicao_contorno)
        if (area > maior_area):
        #if (area > maior_area) and (maior_area >50):
            maior_area = area
            melhor_contorno = posicao_contorno
        #else:
        #draw contour nos objetos avulsos (escolher uma cor: ex: preto)
    #print ("Contorno: "+str(melhor_contorno))
       
    #Parâmetro True/False não são necessários!
    moments = cv2.moments(melhor_contorno)
    area = moments['m00']
    #print("Area do contorno: "+ str(area))
    setServo(servo)
     
    
   #Verificar necessidade
    if area >= minArea:
     x = int (moments['m10'] / moments['m00'])
     y = int (moments['m01'] / moments['m00'])
      
     #demarcar centróide do objeto
     cv2.circle(entrada,(x,y), 5, (0, 255, 0), -1)
      
     # posição do objeto em relação ao centro
     cv2.line(entrada, (int(x),int(y)), (int(centrox),int(centroy)),(0,255,0), 1)
      
     # retangulo do texto
     cv2.rectangle(entrada, (0,107), (160,120), (255,255,255), -1)
      
     # texto da posição no rodapé
     cv2.putText(entrada,"POSICAO: "+str(int(x))+" , "+str(int(y)),(0,117),cv2.FONT_HERSHEY_COMPLEX_SMALL,.6,(0,0,0))
      
     if(area<=120):
        cv2.circle(entrada, (int(x), int(y)), 5, (255, 0, 0), -1)
        Frente()
        #movimento = 'Profundidade para Frente'
     elif(area>=600):
        cv2.circle(entrada, (int(x), int(y)), 5, (0, 0, 255), -1)
        Tras()
        #movimento = 'Profundidade para Tras'
     else:
      Parar()
      limitey = y - centroy
      if (limitey < 0):
       limitey = limitey * -1
       
      limitex = x -centrox
      if (limitex < 0):
       limitex = limitex * -1
 
      if (limitex > limitey):
       ajusteX(x,y, max, centrox)
         
      else:
           
      # Se o limite for maior que a distancia do centro que é considerado centralizado
       if (y - centroy) >= may:
        cv2.line(entrada, (int(x),int(y)), (centrox,centroy),(0,0,255), 1)
        #movimento = 'Vertical Superior'
            servo = servo - 3
        setServo(servo)
         
       # Se o limite for menor ...
       elif (centroy - y) >= may:
        cv2.line(entrada, (int(x),int(y)), (centrox,centroy),(0,0,255), 1)
        #movimento = 'Vertical Inferior'
            servo = servo + 3
        setServo(servo)
         
       #Se o  valor de deslocamento for inferior ao Minimo, faz o minimo
       if servo < servoMin:
            servo = servoMin
        setServo(servo)
         
       # Se o valor de deslocamento for superior ao Maximo, faz o maximo
      if servo > servoMax:
        servo = servoMax
        setServo(servo) 
    
   # Funções do sistema sem nenhum objeto na tela
    else:
     cv2.rectangle(entrada, (0,107), (160,120), (255,255,255), -1)
     cv2.putText(entrada,"Nenhum objeto na tela",(0,117),cv2.FONT_HERSHEY_COMPLEX_SMALL,.5,(0,0,0))
     Parar()
     setServo(servo)
     
    # Resto do loop
    cv2.imshow("Entrada",entrada)
    #cv2.imshow("HSV", imgHSV)
    #cv2.imshow("Thre", imgThresh)
     
    #posicao="Frame "+str(n)+": Posição"
     
    if cv.WaitKey(10) == 27:
        break
    cv.DestroyAllWindows()  
    gpio.cleanup()  
 