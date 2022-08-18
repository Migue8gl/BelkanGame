#include "../Comportamientos_Jugador/jugador.hpp"
#include "motorlib/util.h"

#include <iostream>
#include <cmath>
#include <set>
#include <stack>
#include <queue>
#include <vector>

void rellenaMapa(const Sensores &sensor, vector<vector<unsigned char>> &mapa)
{
	if (sensor.sentido == 0)
	{
		mapa[sensor.posF][sensor.posC] = sensor.terreno[0];
		mapa[sensor.posF - 1][sensor.posC - 1] = sensor.terreno[1];
		mapa[sensor.posF - 1][sensor.posC] = sensor.terreno[2];
		mapa[sensor.posF - 1][sensor.posC + 1] = sensor.terreno[3];
		mapa[sensor.posF - 2][sensor.posC - 2] = sensor.terreno[4];
		mapa[sensor.posF - 2][sensor.posC - 1] = sensor.terreno[5];
		mapa[sensor.posF - 2][sensor.posC] = sensor.terreno[6];
		mapa[sensor.posF - 2][sensor.posC + 1] = sensor.terreno[7];
		mapa[sensor.posF - 2][sensor.posC + 2] = sensor.terreno[8];
		mapa[sensor.posF - 3][sensor.posC - 3] = sensor.terreno[9];
		mapa[sensor.posF - 3][sensor.posC - 2] = sensor.terreno[10];
		mapa[sensor.posF - 3][sensor.posC - 1] = sensor.terreno[11];
		mapa[sensor.posF - 3][sensor.posC] = sensor.terreno[12];
		mapa[sensor.posF - 3][sensor.posC + 1] = sensor.terreno[13];
		mapa[sensor.posF - 3][sensor.posC + 2] = sensor.terreno[14];
		mapa[sensor.posF - 3][sensor.posC + 3] = sensor.terreno[15];
	}
	else if (sensor.sentido == 1)
	{
		mapa[sensor.posF][sensor.posC] = sensor.terreno[0];
		mapa[sensor.posF - 1][sensor.posC + 1] = sensor.terreno[1];
		mapa[sensor.posF][sensor.posC + 1] = sensor.terreno[2];
		mapa[sensor.posF + 1][sensor.posC + 1] = sensor.terreno[3];
		mapa[sensor.posF - 2][sensor.posC + 2] = sensor.terreno[4];
		mapa[sensor.posF - 1][sensor.posC + 2] = sensor.terreno[5];
		mapa[sensor.posF][sensor.posC + 2] = sensor.terreno[6];
		mapa[sensor.posF + 1][sensor.posC + 2] = sensor.terreno[7];
		mapa[sensor.posF + 2][sensor.posC + 2] = sensor.terreno[8];
		mapa[sensor.posF - 3][sensor.posC + 3] = sensor.terreno[9];
		mapa[sensor.posF - 2][sensor.posC + 3] = sensor.terreno[10];
		mapa[sensor.posF - 1][sensor.posC + 3] = sensor.terreno[11];
		mapa[sensor.posF][sensor.posC + 3] = sensor.terreno[12];
		mapa[sensor.posF + 1][sensor.posC + 3] = sensor.terreno[13];
		mapa[sensor.posF + 2][sensor.posC + 3] = sensor.terreno[14];
		mapa[sensor.posF + 3][sensor.posC + 3] = sensor.terreno[15];
	}
	else if (sensor.sentido == 2)
	{
		mapa[sensor.posF][sensor.posC] = sensor.terreno[0];
		mapa[sensor.posF + 1][sensor.posC + 1] = sensor.terreno[1];
		mapa[sensor.posF + 1][sensor.posC] = sensor.terreno[2];
		mapa[sensor.posF + 1][sensor.posC - 1] = sensor.terreno[3];
		mapa[sensor.posF + 2][sensor.posC + 2] = sensor.terreno[4];
		mapa[sensor.posF + 2][sensor.posC + 1] = sensor.terreno[5];
		mapa[sensor.posF + 2][sensor.posC] = sensor.terreno[6];
		mapa[sensor.posF + 2][sensor.posC - 1] = sensor.terreno[7];
		mapa[sensor.posF + 2][sensor.posC - 2] = sensor.terreno[8];
		mapa[sensor.posF + 3][sensor.posC + 3] = sensor.terreno[9];
		mapa[sensor.posF + 3][sensor.posC + 2] = sensor.terreno[10];
		mapa[sensor.posF + 3][sensor.posC + 1] = sensor.terreno[11];
		mapa[sensor.posF + 3][sensor.posC] = sensor.terreno[12];
		mapa[sensor.posF + 3][sensor.posC - 1] = sensor.terreno[13];
		mapa[sensor.posF + 3][sensor.posC - 2] = sensor.terreno[14];
		mapa[sensor.posF + 3][sensor.posC - 3] = sensor.terreno[15];
	}
	else if (sensor.sentido == 3)
	{
		mapa[sensor.posF][sensor.posC] = sensor.terreno[0];
		mapa[sensor.posF + 1][sensor.posC - 1] = sensor.terreno[1];
		mapa[sensor.posF][sensor.posC - 1] = sensor.terreno[2];
		mapa[sensor.posF - 1][sensor.posC - 1] = sensor.terreno[3];
		mapa[sensor.posF + 2][sensor.posC - 2] = sensor.terreno[4];
		mapa[sensor.posF + 1][sensor.posC - 2] = sensor.terreno[5];
		mapa[sensor.posF][sensor.posC - 2] = sensor.terreno[6];
		mapa[sensor.posF - 1][sensor.posC - 2] = sensor.terreno[7];
		mapa[sensor.posF - 2][sensor.posC - 2] = sensor.terreno[8];
		mapa[sensor.posF + 3][sensor.posC - 3] = sensor.terreno[9];
		mapa[sensor.posF + 2][sensor.posC - 3] = sensor.terreno[10];
		mapa[sensor.posF + 1][sensor.posC - 3] = sensor.terreno[11];
		mapa[sensor.posF][sensor.posC - 3] = sensor.terreno[12];
		mapa[sensor.posF - 1][sensor.posC - 3] = sensor.terreno[13];
		mapa[sensor.posF - 2][sensor.posC - 3] = sensor.terreno[14];
		mapa[sensor.posF - 3][sensor.posC - 3] = sensor.terreno[15];
	}
}

estado calcularNuevoObjetivo(const list<estado> &objetivos, const estado &estado_jug)
{
	list<estado>::const_iterator it = objetivos.cbegin();
	list<estado>::const_iterator it2 = objetivos.cbegin();
	double distanciaMin, distancia;

	distanciaMin = sqrt(pow((*it).fila - estado_jug.fila, 2) + pow((*it).columna - estado_jug.columna, 2));

	while (it != objetivos.cend())
	{
		distancia = sqrt(pow((*it).fila - estado_jug.fila, 2) + pow((*it).columna - estado_jug.columna, 2));

		if (distancia < distanciaMin)
		{
			it2 = it;
			distanciaMin = distancia;
		}

		++it;
	}

	return (*it2);
}

// Este es el método principal que se piden en la practica.
// Tiene como entrada la información de los sensores y devuelve la acción a realizar.
// Para ver los distintos sensores mirar fichero "comportamiento.hpp"
Action ComportamientoJugador::think(Sensores sensores)
{
	Action accion = actIDLE;
	estado destino_actual;
	list<estado>::iterator it;

	actual.fila = sensores.posF;
	actual.columna = sensores.posC;
	actual.orientacion = sensores.sentido;

	cout << "Fila: " << actual.fila << endl;
	cout << "Col : " << actual.columna << endl;
	cout << "Ori : " << actual.orientacion << endl;

	if (mapaResultado[actual.fila][actual.columna] == 'K' && !actual.tieneBikini)
	{
		actual.tieneBikini = true;
		if (actual.tieneZapatillas)
			actual.tieneZapatillas = false;
	}
	else if (mapaResultado[actual.fila][actual.columna] == 'D' && !actual.tieneZapatillas)
	{
		actual.tieneZapatillas = true;
		if (actual.tieneBikini)
			actual.tieneBikini = false;
	}

	// Capturo los destinos
	if (objetivos.empty())
	{
		cout << "sensores.num_destinos : " << sensores.num_destinos << endl;
		objetivos.clear();
		for (int i = 0; i < sensores.num_destinos; i++)
		{
			estado aux;
			aux.fila = sensores.destino[2 * i];
			aux.columna = sensores.destino[2 * i + 1];
			objetivos.push_back(aux);
		}
	}

	Action sigAccion;

	if (sensores.nivel != 4)
	{
		if (!hayPlan)
			hayPlan = pathFinding(sensores.nivel, actual, objetivos, plan, sensores.bateria);

		if (hayPlan && plan.size() > 0)
		{
			sigAccion = plan.front();
			plan.erase(plan.begin());
		}
		else
			cout << "no se pudo encontrar un plan\n";
	}
	else
	{
		rellenaMapa(sensores, mapaResultado);

		if (sensores.superficie[2] == 'a')
			sigAccion = actIDLE;

		if ((sensores.terreno[2] == 'P' and plan.front() == actFORWARD) || (sensores.terreno[2] == 'M' and plan.front() == actFORWARD) || (sensores.terreno[2] == 'A' 
		and !actual.tieneBikini and plan.front() == actFORWARD) || (sensores.terreno[2] == 'B' and !actual.tieneZapatillas and plan.front() == actFORWARD))
			hayPlan = false;

		if (!hayPlan || plan.empty())
		{
			hayPlan = pathFinding(sensores.nivel, actual, objetivos, plan, sensores.bateria);
		}

		if (sensores.terreno[0] == 'X' and sensores.bateria < 2500)
			sigAccion = actIDLE;
		else if (hayPlan && plan.size() > 0)
		{
			sigAccion = plan.front();
			plan.erase(plan.begin());
		}
		else
			cout << "no se pudo encontrar un plan\n";

		destino_actual = calcularNuevoObjetivo(objetivos, actual);
		int borrado = false;

		if (destino_actual.fila == actual.fila && destino_actual.columna == actual.columna)
		{
			for (it = objetivos.begin(); it != objetivos.end() && !borrado; ++it)
			{
				if ((*it).fila == destino_actual.fila && (*it).columna == destino_actual.columna)
				{
					objetivos.erase(it);
					borrado = true;
				}
			}
		}

		borrado = false;
	}

	return sigAccion;
}

// Llama al algoritmo de busqueda que se usara en cada comportamiento del agente
// Level representa el comportamiento en el que fue iniciado el agente.
bool ComportamientoJugador::pathFinding(int level, const estado &origen, const list<estado> &destino, list<Action> &plan, int bateria)
{
	estado un_objetivo;
	list<estado>::const_iterator it = destino.cbegin();
	vector<estado> tres_objetivos;
	tres_objetivos.reserve(3);

	switch (level)
	{
	case 0:
		cout << "Demo\n";
		un_objetivo = destino.front();
		cout << "fila: " << un_objetivo.fila << " col:" << un_objetivo.columna << endl;
		return pathFinding_Profundidad(origen, un_objetivo, plan);
		break;

	case 1:
		cout << "Optimo numero de acciones\n";
		un_objetivo = destino.front();
		cout << "fila: " << un_objetivo.fila << " col:" << un_objetivo.columna << endl;
		return pathFinding_Anchura(origen, un_objetivo, plan);
		break;

	case 2:
		cout << "Optimo en coste 1 Objetivo\n";
		un_objetivo = destino.front();
		cout << "fila: " << un_objetivo.fila << " col:" << un_objetivo.columna << endl;
		return pathFinding_CostoUniforme(origen, un_objetivo, plan, bateria);
		break;

	case 3:
		cout << "Optimo en coste 3 Objetivos\n";
		for (int i = 0; i < 3; i++)
		{
			tres_objetivos[i] = *it;
			++it;
		}
		cout << "fila: " << tres_objetivos[0].fila << " col:" << tres_objetivos[0].columna << endl;
		cout << "fila: " << tres_objetivos[1].fila << " col:" << tres_objetivos[1].columna << endl;
		cout << "fila: " << tres_objetivos[2].fila << " col:" << tres_objetivos[2].columna << endl;
		return pathFinding_CostoUniforme3Obj(origen, tres_objetivos, plan, bateria);
		break;
	case 4:
		cout << "Algoritmo de busqueda usado en el reto\n";
		un_objetivo = calcularNuevoObjetivo(destino, origen);
		return pathFinding_CostoUniforme(origen, un_objetivo, plan, bateria);
		break;
	}
	return false;
}

//---------------------- Implementación de la busqueda en profundidad ---------------------------

// Dado el codigo en caracter de una casilla del mapa dice si se puede
// pasar por ella sin riegos de morir o chocar.
bool EsObstaculo(unsigned char casilla)
{
	if (casilla == 'P' or casilla == 'M')
		return true;
	else
		return false;
}

// Comprueba si la casilla que hay delante es un obstaculo. Si es un
// obstaculo devuelve true. Si no es un obstaculo, devuelve false y
// modifica st con la posición de la casilla del avance.
bool ComportamientoJugador::HayObstaculoDelante(estado &st)
{
	int fil = st.fila, col = st.columna;

	// calculo cual es la casilla de delante del agente
	switch (st.orientacion)
	{
	case 0:
		fil--;
		break;
	case 1:
		col++;
		break;
	case 2:
		fil++;
		break;
	case 3:
		col--;
		break;
	}

	// Compruebo que no me salgo fuera del rango del mapa
	if (fil < 0 or fil >= mapaResultado.size())
		return true;
	if (col < 0 or col >= mapaResultado[0].size())
		return true;

	// Miro si en esa casilla hay un obstaculo infranqueable
	if (!EsObstaculo(mapaResultado[fil][col]))
	{
		// No hay obstaculo, actualizo el parametro st poniendo la casilla de delante.
		st.fila = fil;
		st.columna = col;
		return false;
	}
	else
	{
		return true;
	}
}

struct nodo
{
	estado st;
	list<Action> secuencia;
};

struct ComparaEstados
{
	bool operator()(const estado &a, const estado &n) const
	{
		if ((a.fila > n.fila) or (a.fila == n.fila and a.columna > n.columna) or
			(a.fila == n.fila and a.columna == n.columna and a.orientacion > n.orientacion) or
			(a.fila == n.fila and a.columna == n.columna and a.orientacion == n.orientacion and a.tieneBikini > n.tieneBikini) or
			(a.fila == n.fila and a.columna == n.columna and a.orientacion == n.orientacion and a.tieneBikini == n.tieneBikini and a.tieneZapatillas > n.tieneZapatillas) or
			(a.fila == n.fila and a.columna == n.columna and a.orientacion == n.orientacion and a.tieneBikini == n.tieneBikini and a.tieneZapatillas == n.tieneZapatillas and
			 a.encontrado[0] > n.encontrado[0]) or
			(a.fila == n.fila and a.columna == n.columna and a.orientacion == n.orientacion and a.tieneBikini == n.tieneBikini and
			 a.tieneZapatillas == n.tieneZapatillas and a.encontrado[0] == n.encontrado[0] and a.encontrado[1] > n.encontrado[1]) or
			(a.fila == n.fila and a.columna == n.columna and a.orientacion == n.orientacion and a.tieneBikini == n.tieneBikini and
			 a.tieneZapatillas == n.tieneZapatillas and a.encontrado[0] == n.encontrado[0] and a.encontrado[1] == n.encontrado[1] and a.encontrado[2] > n.encontrado[2]))
			return true;
		else
			return false;
	}
};

// Implementación de la busqueda en profundidad.
// Entran los puntos origen y destino y devuelve la
// secuencia de acciones en plan, una lista de acciones.
bool ComportamientoJugador::pathFinding_Profundidad(const estado &origen, const estado &destino, list<Action> &plan)
{
	//Borro la lista
	cout << "Calculando plan\n";
	plan.clear();
	set<estado, ComparaEstados> Cerrados; // Lista de Cerrados
	stack<nodo> Abiertos;				  // Lista de Abiertos

	nodo current;
	current.st = origen;
	current.secuencia.empty();

	Abiertos.push(current);

	while (!Abiertos.empty() and (current.st.fila != destino.fila or current.st.columna != destino.columna))
	{

		Abiertos.pop();
		Cerrados.insert(current.st);

		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion + 1) % 4;
		if (Cerrados.find(hijoTurnR.st) == Cerrados.end())
		{
			hijoTurnR.secuencia.push_back(actTURN_R);
			Abiertos.push(hijoTurnR);
		}

		// Generar descendiente de girar a la izquierda
		nodo hijoTurnL = current;
		hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion + 3) % 4;
		if (Cerrados.find(hijoTurnL.st) == Cerrados.end())
		{
			hijoTurnL.secuencia.push_back(actTURN_L);
			Abiertos.push(hijoTurnL);
		}

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if (!HayObstaculoDelante(hijoForward.st))
		{
			if (Cerrados.find(hijoForward.st) == Cerrados.end())
			{
				hijoForward.secuencia.push_back(actFORWARD);
				Abiertos.push(hijoForward);
			}
		}

		// Tomo el siguiente valor de la Abiertos
		if (!Abiertos.empty())
		{
			current = Abiertos.top();
		}
	}

	cout << "Terminada la busqueda\n";

	if (current.st.fila == destino.fila and current.st.columna == destino.columna)
	{
		cout << "Cargando el plan\n";
		plan = current.secuencia;
		cout << "Longitud del plan: " << plan.size() << endl;
		PintaPlan(plan);
		// ver el plan en el mapa
		VisualizaPlan(origen, plan);
		return true;
	}
	else
	{
		cout << "No encontrado plan\n";
	}

	return false;
}

// Implementación de la busqueda en anchura.
// Entran los puntos origen y destino y devuelve la
// secuencia de acciones en plan, una lista de acciones.
bool ComportamientoJugador::pathFinding_Anchura(const estado &origen, const estado &destino, list<Action> &plan)
{
	//Borro la lista
	cout << "Calculando plan\n";
	plan.clear();
	set<estado, ComparaEstados> Cerrados; // Lista de Cerrados
	queue<nodo> Abiertos;				  // Lista de Abiertos

	nodo current;
	current.st = origen;
	current.secuencia.empty();

	Abiertos.push(current);

	while (!Abiertos.empty() and (current.st.fila != destino.fila or current.st.columna != destino.columna))
	{

		Abiertos.pop();
		Cerrados.insert(current.st);

		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion + 1) % 4;
		if (Cerrados.find(hijoTurnR.st) == Cerrados.end())
		{
			hijoTurnR.secuencia.push_back(actTURN_R);
			Abiertos.push(hijoTurnR);
		}

		// Generar descendiente de girar a la izquierda
		nodo hijoTurnL = current;
		hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion + 3) % 4;
		if (Cerrados.find(hijoTurnL.st) == Cerrados.end())
		{
			hijoTurnL.secuencia.push_back(actTURN_L);
			Abiertos.push(hijoTurnL);
		}

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if (!HayObstaculoDelante(hijoForward.st))
		{
			if (Cerrados.find(hijoForward.st) == Cerrados.end())
			{
				hijoForward.secuencia.push_back(actFORWARD);
				Abiertos.push(hijoForward);
			}
		}

		// Tomo el siguiente valor de la Abiertos
		if (!Abiertos.empty())
		{
			current = Abiertos.front();
		}
	}

	cout << "Terminada la busqueda\n";

	if (current.st.fila == destino.fila and current.st.columna == destino.columna)
	{
		cout << "Cargando el plan\n";
		plan = current.secuencia;
		cout << "Longitud del plan: " << plan.size() << endl;
		PintaPlan(plan);
		// ver el plan en el mapa
		VisualizaPlan(origen, plan);
		return true;
	}
	else
	{
		cout << "No encontrado plan\n";
	}

	return false;
}

void ComportamientoJugador::costeCasilla(estado &c, const string &movement, const vector<vector<unsigned char>> &mapa, int bateria)
{
	char casilla = mapa[c.fila][c.columna];

	switch (casilla)
	{
	case 'A':
		if (movement == "actFORWARD")
		{
			if (c.tieneBikini)
				c.costCasilla = 10;
			else
				c.costCasilla = 200;
		}
		else
		{
			if (c.tieneBikini)
				c.costCasilla = 5;
			else
				c.costCasilla = 500;
		}
		break;

	case 'B':
		if (movement == "actFORWARD")
		{
			if (c.tieneZapatillas)
				c.costCasilla = 15;
			else
				c.costCasilla = 100;
		}
		else
		{
			if (c.tieneZapatillas)
				c.costCasilla = 1;
			else
				c.costCasilla = 3;
		}
		break;

	case 'T':
		c.costCasilla = 2;
		break;

		case 'X':
		if (bateria <= 600)
			c.costCasilla = -10;
		else
			c.costCasilla = 1;
		break;

	default:
		c.costCasilla = 1;
		break;
	}
}

void ComportamientoJugador::costoUniforme(estado &h)
{
	h.costUn += h.costCasilla;
}

struct comparaCosteUniforme
{
	bool operator()(const nodo &n1, const nodo &n2)
	{
		return n1.st.costUn > n2.st.costUn;
	}
};

void ComportamientoJugador::cogerItems(estado &jug)
{
	if (mapaResultado[jug.fila][jug.columna] == 'K' && !jug.tieneBikini)
	{
		jug.tieneBikini = true;
		if (jug.tieneZapatillas)
			jug.tieneZapatillas = false;
	}
	else if (mapaResultado[jug.fila][jug.columna] == 'D' && !jug.tieneZapatillas)
	{
		jug.tieneZapatillas = true;
		if (jug.tieneBikini)
			jug.tieneBikini = false;
	}
}

// Implementación de la busqueda con costo uniforme.
// Entran los puntos origen y destino y devuelve la
// secuencia de acciones en plan, una lista de acciones.
bool ComportamientoJugador::pathFinding_CostoUniforme(const estado &origen, const estado &destino, list<Action> &plan, int bateria)
{
	//Borro la lista
	cout << "Calculando plan\n";
	plan.clear();
	set<estado, ComparaEstados> Cerrados;							   // Lista de Cerrados
	priority_queue<nodo, vector<nodo>, comparaCosteUniforme> Abiertos; // Lista de Abiertos

	nodo current;
	current.st = origen;
	current.secuencia.empty();

	costeCasilla(current.st, "", mapaResultado, bateria);
	current.st.costUn = current.st.costCasilla;

	Abiertos.push(current);

	while (!Abiertos.empty() and (current.st.fila != destino.fila or current.st.columna != destino.columna))
	{
		Abiertos.pop();
		Cerrados.insert(current.st);

		cogerItems(current.st);

		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion + 1) % 4;
		costeCasilla(hijoTurnR.st, "actTURN_R", mapaResultado, bateria);
		costoUniforme(hijoTurnR.st);
		hijoTurnR.secuencia.push_back(actTURN_R);
		Abiertos.push(hijoTurnR);

		// Generar descendiente de girar a la izquierda
		nodo hijoTurnL = current;
		hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion + 3) % 4;
		costeCasilla(hijoTurnL.st, "actTURN_L", mapaResultado, bateria);
		costoUniforme(hijoTurnL.st);
		hijoTurnL.secuencia.push_back(actTURN_L);
		Abiertos.push(hijoTurnL);

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if (!HayObstaculoDelante(hijoForward.st))
		{
			costeCasilla(hijoForward.st, "actFORWARD", mapaResultado, bateria);
			costoUniforme(hijoForward.st);
			hijoForward.secuencia.push_back(actFORWARD);
			Abiertos.push(hijoForward);
		}

		// Tomo el siguiente valor de la Abiertos
		if (!Abiertos.empty())
		{
			current = Abiertos.top();
		}

		while (Cerrados.find(current.st) != Cerrados.end())
		{
			Abiertos.pop();
			current = Abiertos.top();
		}
	}

	cout << "Terminada la busqueda\n";

	if (current.st.fila == destino.fila and current.st.columna == destino.columna)
	{
		cout << "Cargando el plan\n";
		plan = current.secuencia;
		cout << "Longitud del plan: " << plan.size() << endl;
		PintaPlan(plan);
		// ver el plan en el mapa
		VisualizaPlan(origen, plan);
		return true;
	}
	else
	{
		cout << "No encontrado plan\n";
	}

	return false;
}

// Implementación de la busqueda con costo uniforme para 3 objetivos.
// Entran los puntos origen y destino y devuelve la
// secuencia de acciones en plan, una lista de acciones.
bool ComportamientoJugador::pathFinding_CostoUniforme3Obj(const estado &origen, const vector<estado> &destinos, list<Action> &plan, int bateria)
{
	//Borro la lista
	cout << "Calculando plan\n";
	plan.clear();
	set<estado, ComparaEstados> Cerrados;							   // Lista de Cerrados
	priority_queue<nodo, vector<nodo>, comparaCosteUniforme> Abiertos; // Lista de Abiertos

	nodo current;
	bool todosEncontrados = false;
	current.st = origen;
	current.secuencia.empty();

	costeCasilla(current.st, "", mapaResultado, bateria);
	current.st.costUn = current.st.costCasilla;

	Abiertos.push(current);

	while (!Abiertos.empty() and !todosEncontrados)
	{
		Abiertos.pop();
		Cerrados.insert(current.st);

		cogerItems(current.st);

		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion + 1) % 4;
		costeCasilla(hijoTurnR.st, "actTURN_R", mapaResultado, bateria);
		costoUniforme(hijoTurnR.st);
		hijoTurnR.secuencia.push_back(actTURN_R);
		Abiertos.push(hijoTurnR);

		// Generar descendiente de girar a la izquierda
		nodo hijoTurnL = current;
		hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion + 3) % 4;
		costeCasilla(hijoTurnL.st, "actTURN_L", mapaResultado, bateria);
		costoUniforme(hijoTurnL.st);
		hijoTurnL.secuencia.push_back(actTURN_L);
		Abiertos.push(hijoTurnL);

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if (!HayObstaculoDelante(hijoForward.st))
		{
			costeCasilla(hijoForward.st, "actFORWARD", mapaResultado, bateria);
			costoUniforme(hijoForward.st);
			hijoForward.secuencia.push_back(actFORWARD);
			Abiertos.push(hijoForward);
		}

		// Tomo el siguiente valor de la Abiertos
		if (!Abiertos.empty())
		{
			current = Abiertos.top();
		}

		while (Cerrados.find(current.st) != Cerrados.end())
		{
			Abiertos.pop();
			current = Abiertos.top();
		}

		for (int i = 0; i < 3; i++)
			if (current.st.fila == destinos[i].fila && current.st.columna == destinos[i].columna && !current.st.encontrado[i])
				current.st.encontrado[i] = true;

		if (current.st.encontrado[0] && current.st.encontrado[1] && current.st.encontrado[2])
			todosEncontrados = true;
	}

	cout << "Terminada la busqueda\n";

	if (todosEncontrados)
	{
		cout << "Cargando el plan\n";
		plan = current.secuencia;
		cout << "Longitud del plan: " << plan.size() << endl;
		PintaPlan(plan);
		// ver el plan en el mapa
		VisualizaPlan(origen, plan);
		return true;
	}
	else
	{
		cout << "No encontrado plan\n";
	}

	return false;
}

// Sacar por la consola la secuencia del plan obtenido
void ComportamientoJugador::PintaPlan(list<Action> plan)
{
	auto it = plan.begin();
	while (it != plan.end())
	{
		if (*it == actFORWARD)
		{
			cout << "A ";
		}
		else if (*it == actTURN_R)
		{
			cout << "D ";
		}
		else if (*it == actTURN_L)
		{
			cout << "I ";
		}
		else
		{
			cout << "- ";
		}
		it++;
	}
	cout << endl;
}

// Funcion auxiliar para poner a 0 todas las casillas de una matriz
void AnularMatriz(vector<vector<unsigned char>> &m)
{
	for (int i = 0; i < m[0].size(); i++)
	{
		for (int j = 0; j < m.size(); j++)
		{
			m[i][j] = 0;
		}
	}
}

// Pinta sobre el mapa del juego el plan obtenido
void ComportamientoJugador::VisualizaPlan(const estado &st, const list<Action> &plan)
{
	AnularMatriz(mapaConPlan);
	estado cst = st;

	auto it = plan.begin();
	while (it != plan.end())
	{
		if (*it == actFORWARD)
		{
			switch (cst.orientacion)
			{
			case 0:
				cst.fila--;
				break;
			case 1:
				cst.columna++;
				break;
			case 2:
				cst.fila++;
				break;
			case 3:
				cst.columna--;
				break;
			}
			mapaConPlan[cst.fila][cst.columna] = 1;
		}
		else if (*it == actTURN_R)
		{
			cst.orientacion = (cst.orientacion + 1) % 4;
		}
		else
		{
			cst.orientacion = (cst.orientacion + 3) % 4;
		}
		it++;
	}
}

int ComportamientoJugador::interact(Action accion, int valor)
{
	return false;
}
