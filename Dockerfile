#escape=`

# Use microsoft base docker image
FROM mcr.microsoft.com/dotnet/framework/sdk

# Download the Build Tools bootstrapper.
ADD https://aka.ms/vs/16/release/vs_buildtools.exe c:/TEMP/vs_buildtools.exe

# Install MS Build Tools
RUN c:/TEMP/vs_buildtools.exe --quiet --wait --norestart --nocache --installPath C:\BuildTools `
  --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended `
  --add Microsoft.Component.MSBuild `
  --remove Microsoft.VisualStudio.Component.Windows10SDK.10240 `
  --remove Microsoft.VisualStudio.Component.Windows10SDK.10586 `
  --remove Microsoft.VisualStudio.Component.Windows10SDK.14393 `
  --remove Microsoft.VisualStudio.Component.Windows81SDK

WORKDIR c:/Users/ContainerUser/oasis_AO/
ENTRYPOINT ["C:\\BuildTools\\Common7\\Tools\\VsDevCmd.bat", "&&", "powershell.exe", "-NoLogo", "-ExecutionPolicy", "Bypass"]

