#ifndef COMPORTAMIENTOJUGADOR_H
#define COMPORTAMIENTOJUGADOR_H

#include "comportamientos/comportamiento.hpp"

#include <list>
#include <string>

struct estado {
  int fila;
  int columna;
  int orientacion;
  int costCasilla;
  int costUn;
  bool tieneBikini, tieneZapatillas;
  bool encontrado[3] = {false, false, false};
};

class ComportamientoJugador : public Comportamiento {
  public:
    ComportamientoJugador(unsigned int size) : Comportamiento(size) {
      hayPlan = false;
      actual.tieneBikini = false;
      actual.tieneZapatillas = false;
    }
    ComportamientoJugador(std::vector< std::vector< unsigned char> > mapaR) : Comportamiento(mapaR) {
      hayPlan = false;
      actual.tieneBikini = false;
      actual.tieneZapatillas = false;
    }
    ComportamientoJugador(const ComportamientoJugador & comport) : Comportamiento(comport){}
    ~ComportamientoJugador(){}

    Action think(Sensores sensores);
    int interact(Action accion, int valor);
    void VisualizaPlan(const estado &st, const list<Action> &plan);
    ComportamientoJugador * clone(){return new ComportamientoJugador(*this);}

  private:
    // Declarar Variables de Estado
    estado actual;
    list<estado> objetivos;
    list<Action> plan;
    bool hayPlan;

    // Métodos privados de la clase
    bool pathFinding(int level, const estado &origen, const list<estado> &destino, list<Action> &plan, int bateria);
    bool pathFinding_Profundidad(const estado &origen, const estado &destino, list<Action> &plan);
    bool pathFinding_Anchura(const estado &origen, const estado &destino, list<Action> &plan);
    bool pathFinding_CostoUniforme(const estado &origen, const estado &destino, list<Action> &plan, int bateria);
    bool pathFinding_CostoUniforme3Obj(const estado &origen, const vector<estado> &destinos, list<Action> &plan, int bateria);
    void costeCasilla(estado &c, const std::string &movement, const vector<vector<unsigned char>> &mapa, int bateria);
    void costoUniforme(estado &h);
    void cogerItems(estado &jug);

    void PintaPlan(list<Action> plan);
    bool HayObstaculoDelante(estado &st);

};

#endif
