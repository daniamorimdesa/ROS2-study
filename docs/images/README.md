# Imagens da Documentação

Esta pasta contém as imagens utilizadas na documentação do projeto ROS2-study.

## Actions

Para a documentação de Actions (`Actions.md`), as seguintes imagens são utilizadas:

1. **action-client-server-communication.png** ✅
   - Diagrama mostrando a comunicação entre Action Client e Action Server
   - Fluxo: Send Goal → Goal accepted/rejected → Feedback → Result

2. **multiple-action-clients.png** ✅
   - Diagrama mostrando múltiplos Action Clients conectados a um Action Server
   - Ilustra como vários clientes podem enviar goals para o mesmo servidor

3. **goal-state-machine.png** ✅
   - Máquina de estados do ciclo de vida de um goal
   - Estados: ACCEPTED, EXECUTING, CANCELING, SUCCEEDED, CANCELED, ABORTED

## Lifecycle Nodes

Para a documentação de Lifecycle Nodes (`LifecycleNodes.md`), as seguintes imagens são utilizadas:

### Imagens Principais:

1. **lifecycle-nodes-camera-example.png** ✅
   - Exemplo prático de lifecycle com camera driver node
   - Mostra cada estado e o que acontece com a câmera

2. **life_cycle_sm_rso2_documentation.png** ✅
   - Diagrama oficial da documentação ROS2
   - Máquina de estados completa com todas as transições

3. **lifecycle-nodes-state-machine.png** ✅
   - Máquina de estados detalhada
   - Mostra callbacks: on_configure(), on_activate(), on_deactivate(), on_cleanup(), on_shutdown()

### Transições Passo a Passo:

4. **lifecycle-sm-1-configurar-node.png** ✅
   - Transição: Unconfigured → Inactive (configure)

5. **lifecycle-sm-2-ativar-node.png** ✅
   - Transição: Inactive → Active (activate)

6. **lifecycle-sm-3-desativar-node.png** ✅
   - Transição: Active → Inactive (deactivate)

7. **lifecycle-sm-4-limpar-node.png** ✅
   - Transição: Inactive → Unconfigured (cleanup)

8. **lifecycle-sm-5-desligar-node.png** ✅
   - Transição: Qualquer estado → Finalized (shutdown)

## Como adicionar imagens

1. Salve as imagens nesta pasta (`docs/images/`)
2. Use nomes descritivos em kebab-case
3. Prefira formato PNG para diagramas
4. Referencie as imagens nos arquivos Markdown usando:
   ```markdown
   ![Descrição da imagem](images/nome-da-imagem.png)
   ```

## Convenção de nomes

- Use **kebab-case** (palavras separadas por hífen)
- Seja **descritivo** sobre o conteúdo
- Inclua o **contexto** (ex: `lifecycle-`, `action-`, `topic-`)
- Use extensão **.png** para diagramas e gráficos
