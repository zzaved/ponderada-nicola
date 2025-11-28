/*
 * PARTE 1: Navegação Autônoma com Mapa Conhecido
 * Algoritmo: A* (A-Star)
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

// ============================================
// ESTRUTURAS DE DADOS
// ============================================

struct Posicao {
    int linha;
    int coluna;
    
    Posicao(int l = 0, int c = 0) : linha(l), coluna(c) {}
    
    bool operator==(const Posicao& outra) const {
        return linha == outra.linha && coluna == outra.coluna;
    }
    
    struct Hash {
        size_t operator()(const Posicao& p) const {
            return std::hash<int>()(p.linha) ^ (std::hash<int>()(p.coluna) << 1);
        }
    };
};

struct No {
    Posicao pos;
    double g;
    double h;
    double f;
    
    No(Posicao p = Posicao(), double _g = 0, double _h = 0)
        : pos(p), g(_g), h(_h), f(_g + _h) {}
    
    bool operator>(const No& outro) const {
        return f > outro.f;
    }
};

// ============================================
// CLASSE NAVEGADOR
// ============================================

class NavegadorParte1 : public rclcpp::Node {
private:
    std::vector<std::vector<int>> grid_;
    int linhas_;
    int colunas_;
    Posicao robo_pos_;
    Posicao alvo_pos_;
    
    std::vector<std::pair<int, int>> direcoes_ = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1}
    };

public:
    NavegadorParte1() : Node("navegador_parte1") {
        RCLCPP_INFO(this->get_logger(), "Navegador Autonomo - Parte 1");
    }
    
    bool obterMapaManual() {
        RCLCPP_INFO(this->get_logger(), "Obtendo mapa...");
        
        linhas_ = 10;
        colunas_ = 10;
        
        grid_ = {
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
            {1, 2, 0, 0, 1, 0, 0, 0, 0, 1},
            {1, 0, 1, 0, 1, 0, 1, 1, 0, 1},
            {1, 0, 1, 0, 0, 0, 0, 1, 0, 1},
            {1, 0, 1, 1, 1, 1, 0, 1, 0, 1},
            {1, 0, 0, 0, 0, 0, 0, 1, 0, 1},
            {1, 1, 1, 0, 1, 1, 1, 1, 0, 1},
            {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
            {1, 0, 1, 1, 1, 1, 1, 1, 3, 1},
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
        };
        
        for (int i = 0; i < linhas_; i++) {
            for (int j = 0; j < colunas_; j++) {
                if (grid_[i][j] == 2) {
                    robo_pos_ = Posicao(i, j);
                    RCLCPP_INFO(this->get_logger(), "Robo em: (%d, %d)", i, j);
                }
                if (grid_[i][j] == 3) {
                    alvo_pos_ = Posicao(i, j);
                    RCLCPP_INFO(this->get_logger(), "Alvo em: (%d, %d)", i, j);
                }
            }
        }
        
        return true;
    }
    
    double calcularHeuristica(Posicao atual, Posicao destino) {
        return std::abs(atual.linha - destino.linha) + 
               std::abs(atual.coluna - destino.coluna);
    }
    
    bool ehValida(Posicao pos) {
        return pos.linha >= 0 && pos.linha < linhas_ &&
               pos.coluna >= 0 && pos.coluna < colunas_;
    }
    
    bool ehLivre(Posicao pos) {
        if (!ehValida(pos)) return false;
        int valor = grid_[pos.linha][pos.coluna];
        return valor != 1;
    }
    
    std::vector<Posicao> reconstruirCaminho(
        std::unordered_map<Posicao, Posicao, Posicao::Hash>& pais) {
        
        std::vector<Posicao> caminho;
        Posicao atual = alvo_pos_;
        
        while (!(atual == robo_pos_)) {
            caminho.push_back(atual);
            atual = pais[atual];
        }
        caminho.push_back(robo_pos_);
        
        std::reverse(caminho.begin(), caminho.end());
        
        RCLCPP_INFO(this->get_logger(), "Tamanho do caminho: %zu passos", 
                    caminho.size() - 1);
        
        return caminho;
    }
    
    std::vector<Posicao> encontrarCaminho() {
        RCLCPP_INFO(this->get_logger(), "Executando A*...");
        
        std::priority_queue<No, std::vector<No>, std::greater<No>> abertos;
        std::unordered_map<Posicao, bool, Posicao::Hash> fechados;
        std::unordered_map<Posicao, double, Posicao::Hash> custoG;
        std::unordered_map<Posicao, Posicao, Posicao::Hash> pais;
        
        No inicial(robo_pos_, 0, calcularHeuristica(robo_pos_, alvo_pos_));
        abertos.push(inicial);
        custoG[robo_pos_] = 0;
        pais[robo_pos_] = robo_pos_;
        
        int nos_explorados = 0;
        
        while (!abertos.empty()) {
            No atual = abertos.top();
            abertos.pop();
            
            Posicao posAtual = atual.pos;
            
            if (fechados[posAtual]) continue;
            
            fechados[posAtual] = true;
            nos_explorados++;
            
            if (posAtual == alvo_pos_) {
                RCLCPP_INFO(this->get_logger(), "Caminho encontrado!");
                RCLCPP_INFO(this->get_logger(), "Nos explorados: %d", nos_explorados);
                return reconstruirCaminho(pais);
            }
            
            for (auto& dir : direcoes_) {
                Posicao vizinho(
                    posAtual.linha + dir.first,
                    posAtual.coluna + dir.second
                );
                
                if (!ehLivre(vizinho) || fechados[vizinho]) continue;
                
                double novoG = custoG[posAtual] + 1;
                
                if (custoG.find(vizinho) == custoG.end() || 
                    novoG < custoG[vizinho]) {
                    
                    custoG[vizinho] = novoG;
                    pais[vizinho] = posAtual;
                    
                    double h = calcularHeuristica(vizinho, alvo_pos_);
                    abertos.push(No(vizinho, novoG, h));
                }
            }
        }
        
        RCLCPP_ERROR(this->get_logger(), "Caminho nao encontrado!");
        return std::vector<Posicao>();
    }
    
    void imprimirCaminho(const std::vector<Posicao>& caminho) {
        RCLCPP_INFO(this->get_logger(), "\n=== MAPA COM CAMINHO ===");
        
        auto grid_visual = grid_;
        
        for (size_t i = 1; i < caminho.size() - 1; i++) {
            grid_visual[caminho[i].linha][caminho[i].coluna] = 4;
        }
        
        for (int i = 0; i < linhas_; i++) {
            std::string linha_str = "";
            for (int j = 0; j < colunas_; j++) {
                switch(grid_visual[i][j]) {
                    case 0: linha_str += "."; break;
                    case 1: linha_str += "#"; break;
                    case 2: linha_str += "R"; break;
                    case 3: linha_str += "T"; break;
                    case 4: linha_str += "*"; break;
                }
            }
            RCLCPP_INFO(this->get_logger(), "%s", linha_str.c_str());
        }
    }
    
    std::string obterDirecao(Posicao de, Posicao para) {
        int dl = para.linha - de.linha;
        int dc = para.coluna - de.coluna;
        
        if (dl == -1 && dc == 0) return "up";
        if (dl == 1 && dc == 0) return "down";
        if (dl == 0 && dc == -1) return "left";
        if (dl == 0 && dc == 1) return "right";
        
        return "";
    }
    
    void executarCaminhoSimulado(const std::vector<Posicao>& caminho) {
        RCLCPP_INFO(this->get_logger(), "\nEXECUTANDO NAVEGACAO...\n");
        
        for (size_t i = 0; i < caminho.size() - 1; i++) {
            std::string dir = obterDirecao(caminho[i], caminho[i+1]);
            
            RCLCPP_INFO(this->get_logger(), 
                "Passo %zu: (%d,%d) -> %s", 
                i+1, caminho[i].linha, caminho[i].coluna, dir.c_str());
            
            std::this_thread::sleep_for(500ms);
        }
        
        RCLCPP_INFO(this->get_logger(), "\nOBJETIVO ALCANCADO!");
    }
    
    void executar() {
        if (!obterMapaManual()) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao obter mapa!");
            return;
        }
        
        std::this_thread::sleep_for(1s);
        
        auto caminho = encontrarCaminho();
        
        if (caminho.empty()) {
            return;
        }
        
        std::this_thread::sleep_for(1s);
        imprimirCaminho(caminho);
        std::this_thread::sleep_for(2s);
        executarCaminhoSimulado(caminho);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto navegador = std::make_shared<NavegadorParte1>();
    
    RCLCPP_INFO(navegador->get_logger(), "NAVEGACAO AUTONOMA - PARTE 1");
    RCLCPP_INFO(navegador->get_logger(), "Algoritmo: A*\n");
    
    std::this_thread::sleep_for(1s);
    
    navegador->executar();
    
    rclcpp::shutdown();
    return 0;
}
