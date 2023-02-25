import java.lang.ProcessBuilder;

public class OrangePi {
    private final String shutdown_path = "bash shutdown_script";

    public void shutdown(){
        ProcessBuilder shutdown = new ProcessBuilder();
        shutdown.command(shutdown_path);
    }


}