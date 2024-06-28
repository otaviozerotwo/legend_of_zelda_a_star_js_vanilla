/*
  f = g + h

  h é o custo do nó atual até o final
  g é o custo do nó inicial até o atual
*/

function pathTo(node) {
  var curr = node;
  var path = [];

  while (curr.parent) {
    path.unshift(curr);
    curr = curr.parent;
  }

  return path;
}

function getHeap() {
  // Retorna uma nova instância de BinaryHeap, com uma função de pontuação definida
  // A função de pontuação é usada para determinar a ordem dos elementos na BinaryHeap
  // Neste caso, a função de pontuação retorna a propriedade 'f' do nó passado como argumento
  return new BinaryHeap(function(node) {
    return node.f;
  });
}

const astar = {
  // Implementação da função de busca A*
  search: function(graph, start, end, options) {
    // Limpa os nós marcados como sujos no grafo
    graph.cleanDirty();

    // Configurações opcionais para a busca A*
    options = options || {};

    // Heurística a ser usada, padrão é a distância de euclidean
    // var heuristic = options.heuristic || astar.heuristics.euclidean;
    var heuristic = options.heuristic || astar.heuristics.manhattan;

    // Indica se deve retornar o caminho para o nó mais próximo se o destino for inalcançável
    var closest = options.closest || false;
  
    // Cria uma BinaryHeap para manter os nós abertos
    var openHeap = getHeap();

    // Define o nó inicial como o mais próximo, se necessário
    var closestNode = start;
  
    // Calcula a heurística para o nó inicial e marca-o como sujo
    start.h = heuristic(start, end);
    graph.markDirty(start);
  
    // Adiciona o nó inicial à heap aberta
    openHeap.push(start);
  
    // Loop principal da busca A*
    while (openHeap.size() > 0) {
      // Remove o nó com a menor pontuação da heap aberta
      var currentNode = openHeap.pop();

      // console.log('12. Valor currentNode dentro do loop: ', currentNode);
  
      // Se o nó atual for o nó final, retorna o caminho até ele
      if (currentNode === end) {
        
        console.log('13. Caminho pathTo: ', pathTo(currentNode));
        
        return pathTo(currentNode);
      }
  
      // Marca o nó atual como fechado
      currentNode.closed = true;
  
      // Encontra todos os vizinhos do nó atual
      var neighbors = graph.neighbors(currentNode);

      // console.log('14. Valor neighbors: ', neighbors);
  
      // Itera sobre os vizinhos
      for (var i = 0, il = neighbors.length; i < il; ++i) {
        var neighbor = neighbors[i];
  
        // Verifica se o vizinho já foi fechado ou é um obstáculo
        if (neighbor.closed || neighbor.isWall()) {
          continue; // Ignora este vizinho
        }
  
        // Calcula o custo 'g' do vizinho
        var gScore = currentNode.g + neighbor.getCost(currentNode);
        var beenVisited = neighbor.visited;
  
        // Se o vizinho ainda não foi visitado ou o novo custo 'g' é menor
        if (!beenVisited || gScore < neighbor.g) {
          // Marca o vizinho como visitado e atualiza seus atributos
          neighbor.visited = true;
          neighbor.parent = currentNode;
          neighbor.h = neighbor.h || heuristic(neighbor, end);
          neighbor.g = gScore;
          neighbor.f = neighbor.g + neighbor.h;
          // Marca o vizinho como sujo no grafo
          graph.markDirty(neighbor);
          
          // Se a opção closest estiver ativada, atualiza o nó mais próximo
          if (closest) {
            if (neighbor.h < closestNode.h || (neighbor.h === closestNode.h && neighbor.g < closestNode.g)) {
              closestNode = neighbor;
            }
          }
  
          // Adiciona o vizinho à heap aberta se ainda não foi visitado
          if (!beenVisited) {
            openHeap.push(neighbor);
          } else {
            // Caso contrário, atualiza a posição do vizinho na heap
            openHeap.rescoreElement(neighbor);
          }
        }
      }
    }
  
    // Se a opção closest estiver ativada, retorna o caminho para o nó mais próximo
    if (closest) {
      return pathTo(closestNode);
    }

    console.log(pathTo(currentNode));
  
    // Caso contrário, retorna uma lista vazia indicando que o caminho não foi encontrado
    return [];
  },  
  heuristics: {
    // Heurística de distância euclidiana: calcula a distância euclidiana entre duas posições no grid
    // euclidean: function(pos0, pos1) {
      // Calcula a diferença nas coordenadas x e y
      // var dx = pos1.x - pos0.x;
      // var dy = pos1.y - pos0.y;
      // Calcula a distância euclidiana usando o teorema de Pitágoras
      // return Math.sqrt(dx * dx + dy * dy);
    // },
    manhattan: function(pos0, pos1) {
      var d1 = Math.abs(pos1.x - pos0.x);
      var d2 = Math.abs(pos1.y - pos0.y);
      return d1 + d2;
    },
    // diagonal: function(pos0, pos1) {
    //   var D = 1;
    //   var D2 = Math.sqrt(2);
    //   var d1 = Math.abs(pos1.x - pos0.x);
    //   var d2 = Math.abs(pos1.y - pos0.y);
    //   return (D * (d1 + d2)) + ((D2 - (2 * D)) * Math.min(d1, d2));
    // }
  },
  
  // Função para limpar os atributos de um nó
  cleanNode: function(node) {
    // Define todos os atributos do nó como seus valores padrão
    node.f = 0; // Custo total estimado do nó inicial até o nó atual
    node.g = 0; // Custo real do caminho do nó inicial até o nó atual
    node.h = 0; // Estimativa heurística do custo do nó atual até o destino
    node.visited = false; // Indica se o nó já foi visitado durante a busca
    node.closed = false; // Indica se o nó foi fechado durante a busca
    node.parent = null; // Referência ao nó pai no caminho ótimo
  }  
};

// Define uma função construtora chamada BinaryHeap
function BinaryHeap(scoreFunction) {
  // Inicializa um array vazio para armazenar os elementos do heap
  this.content = [];
  // Atribui a função de pontuação fornecida como parâmetro à propriedade scoreFunction do objeto criado
  this.scoreFunction = scoreFunction;
}

// Define métodos para o protótipo de BinaryHeap
BinaryHeap.prototype = {
  // Método para adicionar um elemento ao heap
  push: function(element) {
    // Adiciona o elemento ao final do array content
    this.content.push(element);
    // Reorganiza o heap para manter a propriedade do heap
    this.sinkDown(this.content.length - 1);
  },
  // Método para remover e retornar o elemento com a menor pontuação do heap
  pop: function() {
    // Armazena o elemento raiz (com a menor pontuação)
    var result = this.content[0];
    // Remove o último elemento do array
    var end = this.content.pop();
    // Se ainda houver elementos no heap
    if (this.content.length > 0) {
      // Coloca o último elemento no topo do heap
      this.content[0] = end;
      // Reorganiza o heap para manter a propriedade do heap
      this.bubbleUp(0);
    }
    // Retorna o elemento com a menor pontuação
    return result;
  },
  // Método para remover um nó específico do heap
  remove: function(node) {
    // Encontra a posição do nó no array content
    var i = this.content.indexOf(node);
    // Remove o último elemento do array
    var end = this.content.pop();
    // Se o nó não for o último elemento do heap
    if (i !== this.content.length - 1) {
      // Coloca o último elemento na posição do nó removido
      this.content[i] = end;
      // Reorganiza o heap para manter a propriedade do heap
      if (this.scoreFunction(end) < this.scoreFunction(node)) {
        this.sinkDown(i);
      } else {
        this.bubbleUp(i);
      }
    }
  },
  // Método para retornar o tamanho do heap
  size: function() {
    // Retorna o comprimento do array content, que é o número de elementos no heap
    return this.content.length;
  },
  // Método para reavaliar a posição de um elemento no heap
  rescoreElement: function(node) {
    // Reorganiza o heap com base na posição do nó no array content
    this.sinkDown(this.content.indexOf(node));
  },
  // Método para reorganizar o heap para baixo (mantendo a propriedade do heap)
  sinkDown: function(n) {
    // Obtém o elemento na posição n no array content
    var element = this.content[n];
    // Loop enquanto a posição n não for a raiz do heap
    while (n > 0) {
      // Calcula o índice do pai do elemento na posição n
      var parentN = ((n + 1) >> 1) - 1;
      // Obtém o elemento pai
      var parent = this.content[parentN];
      // Verifica se a pontuação do elemento é menor que a pontuação do pai
      if (this.scoreFunction(element) < this.scoreFunction(parent)) {
        // Se sim, troca o elemento com o pai
        this.content[parentN] = element;
        this.content[n] = parent;
        // Atualiza a posição para o índice do pai
        n = parentN;
      } else {
        // Se não, interrompe o loop
        break;
      }
    }
  },
  // Método para reorganizar o heap para cima (mantendo a propriedade do heap)
  bubbleUp: function(n) {
    // Obtém o comprimento do array content
    var length = this.content.length;
    // Obtém o elemento na posição n no array content
    var element = this.content[n];
    // Obtém a pontuação do elemento
    var elemScore = this.scoreFunction(element);

    // Loop infinito
    while (true) {
      // Calcula os índices dos filhos esquerdo e direito
      var child2N = (n + 1) << 1;
      var child1N = child2N - 1;
      // Variável para armazenar o índice do elemento para troca
      var swap = null;
      var child1Score;

      // Verifica se o filho esquerdo está dentro dos limites do heap
      if (child1N < length) {
        // Obtém o elemento filho esquerdo
        var child1 = this.content[child1N];
        // Obtém a pontuação do filho esquerdo
        child1Score = this.scoreFunction(child1);
        // Verifica se a pontuação do filho esquerdo é menor que a pontuação do elemento
        if (child1Score < elemScore) {
          // Se sim, define o índice do filho esquerdo como índice para troca
          swap = child1N;
        }
      }

      // Verifica se o filho direito está dentro dos limites do heap
      if (child2N < length) {
        // Obtém o elemento filho direito
        var child2 = this.content[child2N];
        // Obtém a pontuação do filho direito
        var child2Score = this.scoreFunction(child2);
        // Verifica se a pontuação do filho direito é menor que a pontuação do elemento ou do filho esquerdo
        if (child2Score < (swap === null ? elemScore : child1Score)) {
          // Se sim, define o índice do filho direito como índice para troca
          swap = child2N;
        }
      }

      // Verifica se ocorreu uma troca
      if (swap !== null) {
        // Realiza a troca entre o elemento na posição n e o elemento na posição swap
        this.content[n] = this.content[swap];
        this.content[swap] = element;
        // Atualiza a posição para o índice da troca
        n = swap;
      } else {
        // Se não ocorreu troca, interrompe o loop
        break;
      }
    }
  }
};

// Define uma função construtora chamada Graph
function Graph(gridIn, options) {
  // Se options for indefinido, define-o como um objeto vazio
  options = options || {};
  // Inicializa um array vazio para armazenar os nós do grafo
  this.nodes = [];
  // this.diagonal = !!options.diagonal;
  // Inicializa um array bidimensional vazio para representar a grade do grafo
  this.grid = [];
  // Percorre as linhas da grade fornecida
  for (var x = 0; x < gridIn.length; x++) {
    // Inicializa uma nova linha na grade do grafo
    this.grid[x] = [];
    // Percorre as colunas da linha atual
    for (var y = 0, row = gridIn[x]; y < row.length; y++) {
      // Cria um novo nó da grade com as coordenadas (x, y) e o peso da célula atual
      // var node = new GridNode(x, y, row[y]);
      var node = new GridNode(x, y, gridIn[x][y]);

      // Adiciona o nó recém-criado à grade do grafo
      this.grid[x][y] = node;

      // console.log(node);

      // Adiciona o nó recém-criado à lista de nós do grafo
      this.nodes.push(node);
    }
  }
  // Inicializa o grafo
  this.init();
}

// Adiciona um método init ao protótipo de Graph
Graph.prototype.init = function() {
  // Inicializa um array vazio para armazenar os nós sujos (dirty)
  this.dirtyNodes = [];
  // Percorre todos os nós do grafo
  for (var i = 0; i < this.nodes.length; i++) {
    // Limpa o nó atual
    astar.cleanNode(this.nodes[i]);
  }
};

// Adiciona um método cleanDirty ao protótipo de Graph
Graph.prototype.cleanDirty = function() {
  // Percorre todos os nós sujos
  for (var i = 0; i < this.dirtyNodes.length; i++) {
    // Limpa o nó sujo atual
    astar.cleanNode(this.dirtyNodes[i]);
  }
  // Limpa o array de nós sujos
  this.dirtyNodes = [];
};

// Adiciona um método markDirty ao protótipo de Graph
Graph.prototype.markDirty = function(node) {
  // Adiciona o nó fornecido ao array de nós sujos
  this.dirtyNodes.push(node);
};

// Adiciona um método neighbors ao protótipo de Graph
Graph.prototype.neighbors = function(node) {
  // Inicializa um array vazio para armazenar os vizinhos do nó
  var ret = [];
  // Obtém as coordenadas x e y do nó
  var x = node.x;
  var y = node.y;
  // Obtém a grade do grafo
  var grid = this.grid;

  // Verifica se existe um nó à esquerda do nó atual
  if (grid[x - 1] && grid[x - 1][y]) {
    // Adiciona o nó à esquerda ao array de vizinhos
    ret.push(grid[x - 1][y]);
  }

  // Verifica se existe um nó à direita do nó atual
  if (grid[x + 1] && grid[x + 1][y]) {
    // Adiciona o nó à direita ao array de vizinhos
    ret.push(grid[x + 1][y]);
  }

  // Verifica se existe um nó acima do nó atual
  if (grid[x] && grid[x][y - 1]) {
    // Adiciona o nó acima ao array de vizinhos
    ret.push(grid[x][y - 1]);
  }

  // Verifica se existe um nó abaixo do nó atual
  if (grid[x] && grid[x][y + 1]) {
    // Adiciona o nó abaixo ao array de vizinhos
    ret.push(grid[x][y + 1]);
  }

  // if (this.diagonal) {
  //   if (grid[x - 1] && grid[x - 1][y - 1]) {
  //     ret.push(grid[x - 1][y - 1]);
  //   }

  //   if (grid[x + 1] && grid[x + 1][y - 1]) {
  //     ret.push(grid[x + 1][y - 1]);
  //   }

  //   if (grid[x - 1] && grid[x - 1][y + 1]) {
  //     ret.push(grid[x - 1][y + 1]);
  //   }

  //   if (grid[x + 1] && grid[x + 1][y + 1]) {
  //     ret.push(grid[x + 1][y + 1]);
  //   }
  // }
  // Retorna o array de vizinhos
  return ret;
};

// Define uma função construtora chamada GridNode
function GridNode(x, y, weight) {
  this.x = x; // Atribui a coordenada x fornecida como parâmetro à propriedade x do objeto criado
  this.y = y; // Atribui a coordenada y fornecida como parâmetro à propriedade y do objeto criado
  this.weight = weight; // Atribui o peso fornecido como parâmetro à propriedade weight do objeto criado
}

// Adiciona um método toString ao protótipo de GridNode
GridNode.prototype.toString = function() {
  // Retorna uma string representando as coordenadas x e y do nó
  return "[" + this.x + " " + this.y + "]";
};

GridNode.prototype.getCost = function() {
  return this.weight;
};

// Adiciona um método isWall ao protótipo de GridNode
GridNode.prototype.isWall = function() {
  // Verifica se o peso do nó é igual a 99 (parede da dungeon)
  return this.weight === 99;
};

document.addEventListener('DOMContentLoaded', () => {
  const grid = [
    [
      99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99, 99,
    ],
    [
      99, 10, 10, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99, 99,
    ],
    [
      99, 10, 10, 10, 10, 10, 10, 10, 10, 10, 99, 99, 10, 10, 10, 99, 99, 99, 10, 10, 10, 99, 99,
      99, 99, 99, 99, 99,
    ],
    [
      99, 10, 10, 99, 99, 10, 99, 99, 99, 10, 10, 10, 10, 10, 10, 99, 99, 99, 10, 10, 10, 99,
      99, 99, 99, 99, 99, 99,
    ],
    [
      99, 99, 99, 99, 99, 10, 99, 99, 99, 10, 99, 99, 10, 10, 10, 99, 99, 99, 10, 10, 10,
      99, 99, 99, 99, 99, 99, 99,
    ],
    [
      99, 99, 99, 99, 99, 10, 99, 99, 99, 10, 99, 99, 99, 99, 99, 99, 99, 99, 99, 10,
      99, 99, 99, 99, 99, 99, 99, 99,
    ],
    [
      99, 99, 99, 99, 99, 10, 99, 99, 99, 10, 99, 99, 99, 99, 99, 99, 99, 99, 99, 10,
      99, 99, 99, 10, 10, 10, 99, 99,
    ],
    [
      99, 99, 99, 10, 10, 10, 99, 99, 99, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
      10, 10, 10, 99, 99,
    ],
    [
      99, 99, 99, 10, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 10, 99, 99, 99, 10,
      99, 99, 99, 10, 10, 10, 99, 99,
    ],
    [
      99, 99, 99, 10, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 10, 99, 99, 99, 10,
      99, 99, 99, 10, 10, 10, 99, 99,
    ],
    [
      99, 99, 99, 10, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 10, 99, 99, 10, 10,
      10, 99, 99, 99, 99, 99, 99, 99,
    ],
    [
      99, 99, 99, 10, 99, 99, 99, 99, 10, 10, 10, 99, 99, 99, 99, 10, 99, 99, 10, 10, 10,
      99, 99, 99, 99, 99, 99, 99,
    ],
    [
      99, 99, 10, 10, 10, 99, 99, 99, 10, 10, 10, 99, 99, 99, 99, 10, 99, 99, 10, 10, 10,
      99, 99, 99, 99, 10, 10, 99,
    ],
    [
      99, 99, 10, 1, 10, 99, 99, 99, 10, 10, 10, 10, 10, 10, 10, 10, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 10, 10, 99,
    ],
    [
      99, 99, 10, 10, 10, 99, 99, 99, 10, 10, 10, 99, 99, 99, 99, 10, 99, 99, 99, 99, 99,
      99, 10, 10, 10, 10, 0, 99,
    ],
    [
      99, 99, 99, 99, 99, 99, 99, 99, 10, 10, 10, 99, 99, 99, 99, 10, 99, 99, 99, 99,
      99, 99, 10, 99, 99, 10, 10, 99,
    ],
    [
      99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 10, 99, 99, 99,
      99, 99, 99, 10, 99, 99, 10, 10, 99,
    ],
    [
      99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 10, 99, 99, 99,
      99, 99, 99, 10, 99, 99, 99, 99, 99,
    ],
    [
      99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 10, 99, 99, 10, 10,
      10, 99, 10, 99, 99, 99, 99, 99,
    ],
    [
      99, 99, 99, 99, 99, 99, 99, 10, 10, 10, 10, 99, 99, 99, 99, 10, 99, 99, 10, 10, 10,
      99, 10, 99, 99, 99, 99, 99,
    ],
    [
      99, 99, 99, 99, 99, 99, 99, 10, 10, 10, 10, 99, 99, 10, 10, 10, 10, 10, 10, 10, 10, 99,
      10, 99, 99, 99, 99, 99,
    ],
    [
      99, 10, 10, 10, 99, 99, 99, 10, 10, 10, 10, 99, 99, 10, 99, 99, 99, 99, 10, 10, 10, 99,
      10, 99, 99, 99, 99, 99,
    ],
    [
      99, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 99, 99, 99, 99, 99, 99, 99, 99,
      10, 99, 99, 99, 99, 99,
    ],
    [
      99, 10, 10, 10, 99, 99, 99, 10, 10, 10, 10, 99, 99, 10, 99, 99, 99, 99, 99, 99, 99,
      99, 10, 99, 99, 99, 99, 99,
    ],
    [
      99, 99, 99, 99, 99, 99, 99, 10, 10, 10, 10, 99, 99, 10, 10, 10, 10, 10, 10, 10, 10, 10,
      10, 99, 99, 99, 99, 99,
    ],
    [
      99, 99, 99, 99, 99, 99, 99, 10, 10, 10, 10, 99, 99, 99, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99,
    ],
    [
      99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99, 99,
    ],
    [
      99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99, 99,
    ],
  ];

  let graph = new Graph(grid);
  let startNode = { x: 14, y: 26 };
  let endNode = { x: 13, y: 3 };
  let custoTotal = 0;
  let celulaAtualIndexIda = 0;
  let celulaAtualIndexVolta = 0;
  let percorrerMapaClicado = false;
  let sairDungeonClicado = false;
  let caminhoIda = [];
  let caminhoVolta = [];
  let celulasPercorridas = grid.map(row => row.map(() => false));

  function atribuirClassNameParaCelula(cell) {
    switch (cell) {
      case 0:
        return "cell-dungeon-value-0";
      case 1:
        return "cell-dungeon-value-pingente-1";
      case 10:
        return "cell-dungeon-value-10";
      case 99:
        return "cell-dungeon-value-99";
      default:
        return "";
    }
  }

  // Atualiza o estado do mapa no DOM
  function atualizarMapa() {
    const container = document.getElementById('mapaDungeonContainer');
    
    if (!container) {
      console.error('Container do mapa não encontrado');
      return;
    }
    container.innerHTML = '';
  
    grid.forEach((row, rowIndex) => {
      const rowDiv = document.createElement('div');
      rowDiv.classList.add('mapa-linha');
      row.forEach((cell, cellIndex) => {
        const cellDiv = document.createElement('div');
        const className = atribuirClassNameParaCelula(cell);
        
        const isCelulaAtualCaminhoIda = celulaAtualIndexIda < (caminhoIda.length - 1) && caminhoIda[celulaAtualIndexIda]?.x === rowIndex && caminhoIda[celulaAtualIndexIda]?.y === cellIndex;
        
        const isCelulaAtualCaminhoVolta = celulaAtualIndexVolta < (caminhoVolta.length - 1) && caminhoVolta[celulaAtualIndexVolta]?.x === rowIndex && caminhoVolta[celulaAtualIndexVolta]?.y === cellIndex;

        const isCelulaPercorrida = celulasPercorridas[rowIndex][cellIndex];
  
        cellDiv.className = `mapa-celula ${className} ${isCelulaPercorrida ? 'mapa-celula-caminho-percorrido' : ''} ${isCelulaAtualCaminhoIda ? 'mapa-celula-posicao-atual' : ''} ${isCelulaAtualCaminhoVolta ? 'mapa-celula-posicao-atual' : ''}`;
        rowDiv.appendChild(cellDiv);
      });
      container.appendChild(rowDiv);
    });
  }

  // Inicia o percurso no mapa
  function iniciarPercurso() {
    percorrerMapaClicado = true;
    const interval = setInterval(() => {
      if (celulaAtualIndexIda < (caminhoIda.length - 1)) {
        const currentNode = caminhoIda[celulaAtualIndexIda];
        celulaAtualIndexIda++;
        custoTotal += currentNode.weight || 1;
        celulasPercorridas[currentNode.x][currentNode.y] = true;
        atualizarMapa();
  
        if (currentNode.x === endNode.x && currentNode.y === endNode.y) {
          clearInterval(interval);
          fimDeJogo = true;
          atualizarResultados();
        }
      } else if (celulaAtualIndexVolta < (caminhoVolta.length - 1)) {
        const currentNode = caminhoVolta[celulaAtualIndexVolta];
        celulaAtualIndexVolta++;
        custoTotal += currentNode.weight || 1;
        celulasPercorridas[currentNode.x][currentNode.y] = true;
        atualizarMapa();
  
        if (currentNode.x === endNode.x && currentNode.y === endNode.y) {
          clearInterval(interval);
          fimDeJogo = true;
          atualizarResultados();
        }
      } else {
        clearInterval(interval);
      } 
    }, 200);
  }
  
  // Atualiza os resultados no DOM
  function atualizarResultados() {
    const resultadosDiv = document.getElementById('resultados');
    resultadosDiv.innerHTML = `<p>Custo Total: ${custoTotal}</p><p>Fim de Jogo: ${fimDeJogo}</p>`;
  }
  
  // Função para iniciar a busca do caminho
  function percorrerMapa() {
    percorrerMapaClicado = true;
  
    caminhoIda = astar.search(graph, graph.grid[startNode.x][startNode.y], graph.grid[endNode.x][endNode.y]);
    fimDeJogo = true;
    
    iniciarPercurso();
  }

  function voltarEntrada() {
    if (caminhoIda.length > 0) {
      celulasPercorridas = Array.from({ length: grid.length }, () => Array(grid[0].length).fill(false));

      percorrerMapaClicado = false;
      sairDungeonClicado = true;

      const novoStartNode = endNode;
      const novoEndNode = startNode;

      // Executa a busca A* e armazena o resultado em caminhoVolta
      caminhoVolta = astar.search(graph, graph.grid[novoStartNode.x][novoStartNode.y], graph.grid[novoEndNode.x][novoEndNode.y]);

      // Atualiza os nós de início e fim
      // startNode = { x: 39, y: 17 };
      // endNode = { x: 6, y: 5 };

      iniciarPercurso();
    } else {
      alert('Colete o pingente no final da dungeon');
    }
  }

  function sairDungeon() {
    if (sairDungeonClicado) {
      window.location.href = '../../MapaPrincipal/mapaPrincipal.html'
    } else {
      alert('Para sair é necessário pegar o pingente e retornar para a entrada');
    }
  }

  document.getElementById('percorrerMapa').addEventListener('click', percorrerMapa);
  document.getElementById('voltarEntrada').addEventListener('click', voltarEntrada);
  document.getElementById('sairDungeon').addEventListener('click', sairDungeon);

  // Função inicial para configurar a grid e mostrar no DOM
  atualizarMapa();
});