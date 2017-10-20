import java.io.IOException;

abstract class SearchAlgorithm {
    Grid g;

    SearchAlgorithm (Grid grid) {
        g = grid;
    }

    /**
     * Onde comeca em cm
     */
    abstract public void setEnd (Double x, Double y);

    /**
     * Onde termina em cm
     */
    abstract public void setInit (Double x, Double y);

    /**
     * monta o caminho e retorna se for possivel ou nao
     * retorna o custo (numeros de poses que tera que andar)
     */
    abstract public int search ();

    /**
     * Salva a imagem com o nome name
     */
    public void save (String name) {
        try {
            g.savePNG (name, 4);
        } catch (IOException e) {
            System.out.println("Error to save image.");
        }

    }

}