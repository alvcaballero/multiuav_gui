# mermaid
# Secuencia send task
sequenceDiagram
    GridWatch->>+GCS: Send Task
    GCS-->GCS: Planing mission
    GCS->>-GridWatch: Mission to sent to UAV
    GCS->>+UAV: load mission 
    UAV->>-GCS: Mission loaded
    GCS->>+UAV: Command Mission
    UAV->>-GCS: Command Mission OK
    UAV-->UAV: doing mission
    UAV->>GCS: Finish mission
    alt Can be automatic into UAV
        GCS->>+UAV: Download files from Autopilot
        UAV->>-GCS: Download downloaded
        UAV-->UAV:  download files of mision
    end
    UAV->>GCS: FinishGetFiles
    GCS-->GCS: SFTP download  files from UAV
    GCS->>GridWatch: Ready for Send Result mission
    GridWatch->>+GCS: Donwload information
    GCS->>-GridWatch: success
