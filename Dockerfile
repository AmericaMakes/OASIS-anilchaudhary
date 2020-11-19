# Sample Dockerfile

# Indicates that the windowsservercore image will be used as the base image.
FROM mcr.microsoft.com/dotnet/framework/sdk

ADD ["./OASIS baseline source", "c:/Users/ContainerUser/"]
ADD ./compileBaseCode.bat c:/Users/ContainerUser/
# Download the Build Tools bootstrapper.
ADD https://aka.ms/vs/16/release/vs_buildtools.exe c:/TEMP/vs_buildtools.exe

# Install Build Tools with the Microsoft.VisualStudio.Workload.AzureBuildTools workload, excluding workloads and components with known issues.
RUN c:/TEMP/vs_buildtools.exe --quiet --wait --norestart --nocache --installPath C:\BuildTools --add Microsoft.VisualStudio.Workload.AzureBuildTools --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended --add Microsoft.Component.MSBuild --remove Microsoft.VisualStudio.Component.Windows10SDK.10240 --remove Microsoft.VisualStudio.Component.Windows10SDK.10586 --remove Microsoft.VisualStudio.Component.Windows10SDK.14393  --remove Microsoft.VisualStudio.Component.Windows81SDK

WORKDIR c:/Users/ContainerUser/
ENTRYPOINT ["C:\\BuildTools\\Common7\\Tools\\VsDevCmd.bat", "&&", "powershell.exe", "-NoLogo", "-ExecutionPolicy", "Bypass"]

